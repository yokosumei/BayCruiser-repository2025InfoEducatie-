from flask import Flask, render_template, Response, request, redirect, url_for, jsonify
from ultralytics import YOLO
from picamera2 import Picamera2
import RPi.GPIO as GPIO
import numpy as np
import threading
import cv2
import time
import atexit
import os

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
output_frame = None
detected_flag = False
popup_sent = False
last_detection_time = 0

def cleanup():
    servo1.stop()
    servo2.stop()
    GPIO.cleanup()

atexit.register(cleanup)

def activate_servos():
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

def detect_objects():
    global output_frame, streaming, detected_flag, popup_sent, last_detection_time

    cam_x = 640 // 2
    cam_y = 480 // 2
    PIXELS_PER_CM = 10

    while streaming:
        frame = picam2.capture_array()
        results = model(frame, verbose=False)
        names = results[0].names
        class_ids = results[0].boxes.cls.tolist()

        detected = False

        for i, cls_id in enumerate(class_ids):
            if names[int(cls_id)] == "om_la_inec":
                detected = True
                detected_flag = True
                popup_sent = True
                last_detection_time = time.time()
                activate_servos()

                # Extrage boxul
                box = results[0].boxes[i]
                x1, y1, x2, y2 = map(int, box.xyxy[0])
                obj_x = (x1 + x2) // 2
                obj_y = (y1 + y2) // 2

                dx_cm = (obj_x - cam_x) / PIXELS_PER_CM
                dy_cm = (obj_y - cam_y) / PIXELS_PER_CM
                dist_cm = (dx_cm**2 + dy_cm**2)**0.5

                # Deseneare
                cv2.line(frame, (cam_x, cam_y), (obj_x, obj_y), (0, 0, 255), 2)
                cv2.circle(frame, (cam_x, cam_y), 5, (255, 0, 0), -1)
                cv2.circle(frame, (obj_x, obj_y), 5, (0, 255, 0), -1)

                # afisare rezultate
                offset_text = f"Δx: {dx_cm:.1f} cm, Δy: {dy_cm:.1f} cm"
                dist_text = f"Dist: {dist_cm:.1f} cm"
                cv2.putText(frame, offset_text, (20, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0,0,0), 2)
                cv2.putText(frame, offset_text, (20, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0,255,255), 1)
                cv2.putText(frame, dist_text, (20, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0,0,0), 2)
                cv2.putText(frame, dist_text, (20, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0,255,255), 1)

                break  # doar primul om detectat

        if not detected and time.time() - last_detection_time > 5:
            detected_flag = False
            popup_sent = False

        annotated = results[0].plot()
        with lock:
            output_frame = cv2.imencode('.jpg', frame)[1].tobytes()

        time.sleep(0.05)


    

@app.route("/", methods=["GET", "POST"])
def index():
    if request.method == "POST":
        if 'file' not in request.files:
            return render_template("index.html", error="No file")
        file = request.files['file']
        if file.filename == '':
            return render_template("index.html", error="No filename")

        input_path = os.path.join(app.config['UPLOAD_FOLDER'], 'input.mp4')
        output_path = os.path.join(app.config['UPLOAD_FOLDER'], 'result.mp4')
        file.save(input_path)

        results = model.predict(
            source=input_path,
            save=True,
            save_txt=False,
            project=app.config['UPLOAD_FOLDER'],
            name="processed",
            exist_ok=True,
            stream=True  
        )

        processed_dir = os.path.join(app.config['UPLOAD_FOLDER'], "processed")
        for fname in os.listdir(processed_dir):
            if fname.endswith(".avi") or fname.endswith(".mp4"):
                os.rename(os.path.join(processed_dir, fname), output_path)
                break

        return render_template("index.html", video_uploaded=True)

    return render_template("index.html", video_uploaded=False)


@app.route("/video_feed")
def video_feed():
    def generate():
        global output_frame
        while True:
            if not streaming:
                time.sleep(0.1)
                continue
            with lock:
                frame = output_frame if output_frame is not None else blank_frame()
            yield (b"--frame\r\n"
                   b"Content-Type: image/jpeg\r\n\r\n" + frame + b"\r\n")
            time.sleep(0.05)
    return Response(generate(), mimetype="multipart/x-mixed-replace; boundary=frame")

@app.route("/start_stream")
def start_stream():
    global streaming
    if not streaming:
        streaming = True
        thread = threading.Thread(target=detect_objects)
        thread.daemon = True
        thread.start()
    return jsonify({"status": "started"})

@app.route("/stop_stream")
def stop_stream():
    global streaming
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
    app.run(host="0.0.0.0", port=5000)
