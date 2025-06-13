from flask import Flask, render_template, Response, request, jsonify
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

streaming = False
lock = threading.Lock()
output_lock = threading.Lock()
frame_buffer = None
output_frame = None
yolo_output_frame = None
detected_flag = False
popup_sent = False
last_detection_time = 0
detection_frame_skip = 2  # număr de frame-uri de sărit
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

def camera_thread():
    global frame_buffer
    logging.info("Firul principal (camera) a pornit.")
    while True:
        frame = picam2.capture_array()
        with lock:
            frame_buffer = frame.copy()
        time.sleep(0.01)

def detection_thread():
    global frame_buffer, yolo_output_frame, detected_flag, popup_sent, last_detection_time, frame_counter
    cam_x, cam_y = 320, 240
    PIXELS_PER_CM = 10
    object_present = False
    logging.info("Firul 2 (detectie) a pornit.")
    while True:
        if not streaming:
            time.sleep(0.1)
            continue
        frame_counter += 1
        if frame_counter % detection_frame_skip != 0:
            time.sleep(0.01)
            continue
        with lock:
            frame = frame_buffer.copy() if frame_buffer is not None else None
        if frame is None:
            logging.warning("Nu există frame pentru detecție.")
            time.sleep(0.05)
            continue

        results = model(frame, verbose=False)
        annotated = results[0].plot()

        names = results[0].names
        class_ids = results[0].boxes.cls.tolist()
        current_detection = False

        for i, cls_id in enumerate(class_ids):
            if names[int(cls_id)] == "om_la_inec":
                current_detection = True

                if not object_present:
                    detected_flag = True
                    popup_sent = True
                    last_detection_time = time.time()
                    object_present = True

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
                cv2.putText(annotated, offset_text, (20, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0,255,255), 2)
                cv2.putText(annotated, dist_text, (20, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0,255,255), 2)
                break

        if not current_detection:
            detected_flag = False
            popup_sent = False
            object_present = False

        with output_lock:
            yolo_output_frame = cv2.imencode('.jpg', annotated)[1].tobytes()

        time.sleep(0.01)

def stream_thread():
    global output_frame
    logging.info("Firul 3 (livrare frame) a pornit.")
    while True:
        if not streaming:
            time.sleep(0.1)
            continue
        with lock:
            frame = frame_buffer.copy() if frame_buffer is not None else None
        if frame is None:
            time.sleep(0.05)
            continue
        jpeg = cv2.imencode('.jpg', frame)[1].tobytes()
        with output_lock:
            output_frame = jpeg
        time.sleep(0.05)

@app.route("/")
def index():
    return render_template("index.html")

@app.route("/video_feed")
def video_feed():
    def generate():
        global output_frame
        logging.info("Client conectat la /video_feed")
        while True:
            if not streaming:
                time.sleep(0.1)
                continue
            with output_lock:
                frame = output_frame if output_frame is not None else blank_frame()
            yield (b"--frame\r\n"
                   b"Content-Type: image/jpeg\r\n\r\n" + frame + b"\r\n")
            time.sleep(0.05)
    return Response(generate(), mimetype="multipart/x-mixed-replace; boundary=frame")

@app.route("/yolo_feed")
def yolo_feed():
    def generate():
        global yolo_output_frame
        logging.info("Client conectat la /yolo_feed")
        while True:
            if not streaming:
                time.sleep(0.1)
                continue
            with output_lock:
                frame = yolo_output_frame if yolo_output_frame is not None else blank_frame()
            yield (b"--frame\r\n"
                   b"Content-Type: image/jpeg\r\n\r\n" + frame + b"\r\n")
            time.sleep(0.05)
    return Response(generate(), mimetype="multipart/x-mixed-replace; boundary=frame")

@app.route("/start_stream")
def start_stream():
    global streaming
    if not streaming:
        logging.info("Stream pornit de utilizator")
        streaming = True
    return jsonify({"status": "started"})

@app.route("/stop_stream")
def stop_stream():
    global streaming
    logging.info("Stream oprit de utilizator")
    streaming = False
    return jsonify({"status": "stopped"})

@app.route("/detection_status")
def detection_status():
    return jsonify({"detected": popup_sent})

@app.route("/misca")
def activate():
    activate_servos()
    return "Servomotor activat"

@app.route("/takeoff")
def takeoff():
    return "Drone Takeoff (dezactivat temporar)"

@app.route("/land")
def land():
    return "Drone Landing (dezactivat temporar)"

if __name__ == "__main__":
    threading.Thread(target=camera_thread, name="CameraThread", daemon=True).start()
    threading.Thread(target=detection_thread, name="DetectionThread", daemon=True).start()
    threading.Thread(target=stream_thread, name="StreamThread", daemon=True).start()
    logging.info("Pornire server Flask")
    app.run(host="0.0.0.0", port=5000)
