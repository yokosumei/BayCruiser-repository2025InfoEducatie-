from flask import Flask, render_template, send_file, Response, request, jsonify
from ultralytics import YOLO
from picamera2 import Picamera2
import RPi.GPIO as GPIO
import numpy as np
import threading
import cv2
import time
import atexit
import os
import logging
from collections import deque
from dronekit import connect, LocationGlobalRelative
from collections import deque

# buffer FIFO pentru output frame
output_frame_buffer = deque(maxlen=1)
yolo_output_frame_buffer = deque(maxlen=1)

# === CONFIGURARE LOGGING ===
logging.basicConfig(level=logging.DEBUG, format='[%(levelname)s] (%(threadName)s) %(message)s')

app = Flask(__name__)
app.config['UPLOAD_FOLDER'] = 'static/uploads'
os.makedirs(app.config['UPLOAD_FOLDER'], exist_ok=True)

model = YOLO("my_model.pt")
picam2 = Picamera2()
picam2.configure(picam2.create_video_configuration(main={"format": "RGB888", "size": (640, 480)}))
picam2.start()

GPIO.setmode(GPIO.BOARD)
GPIO.setup(11, GPIO.OUT)
GPIO.setup(12, GPIO.OUT)
servo1 = GPIO.PWM(11, 50)
servo2 = GPIO.PWM(12, 50)
servo1.start(7.5)
servo2.start(7.5)
time.sleep(0.3)
servo1.ChangeDutyCycle(0)
servo2.ChangeDutyCycle(0)

streaming = True
stream_lock = threading.Lock()
frame_lock = threading.Lock()

output_lock = threading.Lock()
frame_buffer = None
yolo_output_frame = None
detected_flag = False
popup_sent = False
last_detection_time = 0
frame_counter = 0

def cleanup():
    servo1.stop()
    servo2.stop()
    GPIO.cleanup()
  

atexit.register(cleanup)
   
   



def activate_servos():
    logging.debug("Activare servomotoare")
    servo1.ChangeDutyCycle(12.5)
    servo2.ChangeDutyCycle(2.5)
    time.sleep(0.3)
    servo1.ChangeDutyCycle(0)
    servo2.ChangeDutyCycle(0)
    time.sleep(2)
    servo1.ChangeDutyCycle(7.5)
    servo2.ChangeDutyCycle(7.5)
    time.sleep(0.3)
    servo1.ChangeDutyCycle(0)
    servo2.ChangeDutyCycle(0)

def blank_frame():
    img = np.zeros((480, 640, 3), dtype=np.uint8)
    _, buffer = cv2.imencode('.jpg', img)
    return buffer.tobytes()


# === GPS SIMULATOR SAU REAL ===
USE_SIMULATOR = True
popup_sent=False

class GPSValue:
    def __init__(self, lat, lon, alt):
        self.lat = lat
        self.lon = lon
        self.alt = alt

class BaseGPSProvider:
    def get_location(self):
        raise NotImplementedError()
    def close(self):
        pass

class MockGPSProvider:
    def __init__(self):
        self.coordinates = deque([
            (44.4391 + i * 0.0001, 26.0961 + i * 0.0001, 80.0) for i in range(20)
        ])

    def get_location(self):
        try:
            lat, lon, alt = self.coordinates[0]
            self.coordinates.rotate(-1)
            logging.debug(f"[MOCK GPS] Coordonată returnată: lat={lat}, lon={lon}, alt={alt}")
            return GPSValue(lat, lon, alt)
        except Exception as e:
            logging.exception("[MOCK GPS] Eroare la generarea coordonatei")
            return GPSValue(None, None, None)

class DroneKitGPSProvider(BaseGPSProvider):
    def __init__(self):
        self.vehicle = connect('/dev/ttyUSB0', wait_ready=True, baud=57600)
        self.location = GPSValue(None, None, None)
        self.vehicle.add_attribute_listener('location.global_frame', self.gps_callback)

    def gps_callback(self, self_ref, attr_name, value):
        self.location = GPSValue(value.lat, value.lon, value.alt)

    def get_location(self):
        return self.location

    def close(self):
        self.vehicle.close()

gps_provider = MockGPSProvider() if USE_SIMULATOR else DroneKitGPSProvider()

def camera_thread():
    global frame_buffer, output_frame_buffer
    logging.info("[CAMERA] Firul principal a pornit.")
    while True:
        try:
            frame = picam2.capture_array()
            gps = gps_provider.get_location()
            gps_snapshot = {
                "lat": gps.lat,
                "lon": gps.lon,
                "alt": gps.alt,
                "timestamp": time.time()
            }

            if gps.lat is not None and gps.lon is not None:
                cv2.putText(frame,
                            f"Lat: {gps.lat:.6f} Lon: {gps.lon:.6f} Alt: {gps.alt:.1f}",
                            (10, 20),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)

            encoded = cv2.imencode('.jpg', frame)[1].tobytes()
            with stream_lock:
                output_frame_buffer.append(encoded)
            with frame_lock:
                frame_buffer = {
                    "image": frame.copy(),
                    "gps": gps_snapshot
                }
            logging.debug(f"[STREAM] Frame capturat la {gps_snapshot['timestamp']:.3f} transmis la {time.time():.3f}")
        except Exception as e:
            logging.exception("[CAMERA] Eroare în bucla principală")

def detection_thread():
    global frame_buffer, yolo_output_frame_buffer
    cam_x, cam_y = 320, 240
    PIXELS_PER_CM = 10
    object_present = False
    frame_counter = 0
    detection_frame_skip = 2
    logging.info("[DETECTIE] Firul de detecție a pornit.")

    while True:
        if not streaming:
            time.sleep(0.1)
            continue

        frame_counter += 1
        if frame_counter % detection_frame_skip != 0:
            time.sleep(0.1)
            logging.warning("[DETECTIE] Frame rejectat...............................................")
            frame_counter=0;
            continue

        with frame_lock:
            data = frame_buffer.copy() if frame_buffer else None
        if data is None:
            logging.warning("[DETECTIE] Nu există frame pentru detecție.")
           # time.sleep(0.01)
            continue

        frame = data["image"]
        gps_info = data["gps"]

        resized = cv2.resize(frame, (320, 240))  # Micșorare doar pentru detecție

        results = model(resized, verbose=False)
        annotated = frame.copy()  # folosim imaginea originală pentru desen

        if gps_info["lat"] is not None and gps_info["lon"] is not None:
            gps_text = f"Lat: {gps_info['lat']:.6f} Lon: {gps_info['lon']:.6f} Alt: {gps_info['alt']:.1f}"
            timestamp_text = f"Timp: {time.strftime('%H:%M:%S', time.localtime(gps_info['timestamp']))}"
            cv2.putText(annotated, gps_text, (10, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
            cv2.putText(annotated, timestamp_text, (10, 45), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)

        names = results[0].names
        class_ids = results[0].boxes.cls.tolist()
        current_detection = False

        for i, cls_id in enumerate(class_ids):
            if names[int(cls_id)] == "om_la_inec":
                current_detection = True

                box = results[0].boxes[i]
                x1, y1, x2, y2 = map(int, box.xyxy[0])
                obj_x = (x1 + x2) // 2
                obj_y = (y1 + y2) // 2

                dx_cm = (obj_x - cam_x) / PIXELS_PER_CM
                dy_cm = (obj_y - cam_y) / PIXELS_PER_CM
                dist_cm = (dx_cm**2 + dy_cm**2)**0.5

                cv2.line(annotated, (cam_x, cam_y), (obj_x, obj_y), (0, 0, 255), 2)
                cv2.circle(annotated, (cam_x, cam_y), 5, (255, 0, 0), -1)
                cv2.circle(annotated, (obj_x, obj_y), 5, (0, 255, 0), -1)

                offset_text = f"x:{dx_cm:.1f}cm | y:{dy_cm:.1f}cm"
                dist_text = f"Dist: {dist_cm:.1f}cm"
                cv2.putText(annotated, offset_text, (20, 70), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0,255,255), 2)
                cv2.putText(annotated, dist_text, (20, 100), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0,255,255), 2)
                break
        try:
            encoded = cv2.imencode('.jpg', annotated)[1].tobytes()
            with output_lock:
                yolo_output_frame_buffer.append(encoded)
            logging.debug(f"[YOLO] Frame detectat la {gps_info['timestamp']:.3f} transmis la {time.time():.3f}")
        except Exception as e:
            logging.exception("[YOLO] Eroare la codificarea frame-ului YOLO")



@app.route("/")
def index():
    return render_template("index.html")

@app.route("/video_feed")
def video_feed():
    def generate():
        logging.info("[FLASK] Client conectat la /video_feed")
        while True:
            try:
                if not streaming:
                    logging.debug("[FLASK] Streaming oprit — se așteaptă pornirea streamului.")
                    time.sleep(0.1)
                    continue
                with output_lock:
                    frame = output_frame_buffer[-1] if output_frame_buffer else blank_frame()
                yield (b"--frame\r\n"
                       b"Content-Type: image/jpeg\r\n\r\n" + frame + b"\r\n")
                logging.debug("[FLASK] Frame trimis către client /video_feed")
            except Exception as e:
                logging.exception("[FLASK] Eroare în fluxul video_feed")
                break
    try:
        return Response(generate(), mimetype="multipart/x-mixed-replace; boundary=frame")
    except Exception as e:
        logging.exception("[FLASK] Eroare la inițializarea fluxului video_feed")
        return "Eroare la inițializarea fluxului video_feed", 500


@app.route("/yolo_feed")
def yolo_feed():
    def generate():
        logging.info("[FLASK] Client conectat la /yolo_feed")
        while True:
            try:
                if not streaming:
                    logging.debug("[FLASK] Streaming oprit — se așteaptă pornirea streamului YOLO.")
                    time.sleep(0.1)
                    continue
                with output_lock:
                    frame = yolo_output_frame_buffer[-1] if yolo_output_frame_buffer else blank_frame()
                yield (b"--frame\r\n"
                       b"Content-Type: image/jpeg\r\n\r\n" + frame + b"\r\n")
                logging.debug("[FLASK] Frame trimis către client /yolo_feed")
            except Exception as e:
                logging.exception("[FLASK] Eroare în fluxul yolo_feed")
                break
    try:
        return Response(generate(), mimetype="multipart/x-mixed-replace; boundary=frame")
    except Exception as e:
        logging.exception("[FLASK] Eroare la inițializarea fluxului yolo_feed")
        return "Eroare la inițializarea fluxului yolo_feed", 500

        
        
@app.route("/yolo_feed_snapshot")
def yolo_feed_snapshot():
    global yolo_output_frame
    with output_lock:
        frame = yolo_output_frame if yolo_output_frame is not None else blank_frame()
    return Response(frame, mimetype='image/jpeg') 

@app.route("/start_stream")
def start_stream():
    global streaming
    try:
        logging.info("[FLASK] /start_stream apelat")
        streaming = True
        return jsonify({"status": "started"})
    except Exception as e:
        logging.exception("[FLASK] Eroare la pornirea streamului")
        return jsonify({"status": "error", "message": str(e)})

@app.route("/stop_stream")
def stop_stream():
    global streaming
    try:
        logging.info("[FLASK] /stop_stream apelat")
        streaming = False
        return jsonify({"status": "stopped"})
    except Exception as e:
        logging.exception("[FLASK] Eroare la oprirea streamului")
        return jsonify({"status": "error", "message": str(e)})

@app.route("/detection_status")
def detection_status():
    try:
        logging.debug("[FLASK] /detection_status apelat")
        return jsonify({"detected": popup_sent})
    except Exception as e:
        logging.exception("[FLASK] Eroare la /detection_status")
        return jsonify({"status": "error", "message": str(e)})

@app.route("/misca")
def activate():
    try:
        logging.info("[FLASK] /misca apelat — se activează servomotorul")
        activate_servos()
        return "Servomotor activat"
    except Exception as e:
        logging.exception("[FLASK] Eroare la activarea servomotorului")
        return "Eroare la activare servo", 500


@app.route("/takeoff")
def takeoff():
    return "Drone Takeoff (dezactivat temporar)"

@app.route("/land")
def land():
    return "Drone Landing (dezactivat temporar)"

@app.route("/return_to_event")
def return_to_event():
    global event_location
    logging.info("[FLASK] /return_to_event apelat")
    if not event_location:
        logging.warning("[FLASK] Nu există coordonate salvate pentru revenirea dronei.")
        return jsonify({"status": "no event location"})
    try:
        logging.info(f"[FLASK] Se trimite drona înapoi la: {event_location}")
        gps_provider.vehicle.simple_goto(event_location)
        return jsonify({
            "status": "returning",
            "lat": event_location.lat,
            "lon": event_location.lon,
            "alt": event_location.alt
        })
    except Exception as e:
        logging.exception("[FLASK] Eroare la trimiterea dronei către locație")
        return jsonify({"status": "error", "message": str(e)})

if __name__ == "__main__":
    threading.Thread(target=camera_thread, name="CameraThread", daemon=True).start()
    threading.Thread(target=detection_thread, name="DetectionThread", daemon=True).start()
    logging.info("Pornire server Flask")
    app.run(host="0.0.0.0", port=5000, threaded=True)
