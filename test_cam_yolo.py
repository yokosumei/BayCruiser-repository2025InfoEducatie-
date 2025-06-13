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
import logging

# === Log Setup ===
logging.basicConfig(level=logging.DEBUG, format='[%(asctime)s] %(threadName)s: %(message)s')

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
frame_buffer = None
annotated_frame = None
detected_flag = False
popup_sent = False
last_detection_time = 0


def cleanup():
    servo1.stop()
    servo2.stop()
    GPIO.cleanup()

atexit.register(cleanup)


def activate_servos():
    logging.debug("Activating servos")
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


def capture_camera():
    global frame_buffer, streaming
    logging.debug("Started camera capture thread")
    while True:
        if streaming:
            frame = picam2.capture_array()
            with lock:
                frame_buffer = frame.copy()
            logging.debug("Frame captured")
        time.sleep(0.03)


def detect_objects():
    global annotated_frame, frame_buffer, detected_flag, popup_sent, last_detection_time
    logging.debug("Started detection thread")
    cam_x = 320
    cam_y = 240
    PIXELS_PER_CM = 10
    object_present = False
    while True:
        if streaming:
            with lock:
                frame = frame_buffer.copy() if frame_buffer is not None else None
            if frame is None:
                time.sleep(0.01)
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
                        logging.debug("Object detected")
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
                if object_present:
                    logging.debug("Object lost")
                detected_flag = False
                popup_sent = False
                object_present = False
            with lock:
                annotated_frame = annotated.copy()
        time.sleep(0.05)


def stream_output():
    global output_frame, annotated_frame
    logging.debug("Started stream output thread")
    while True:
        if streaming:
            with lock:
                frame = annotated_frame.copy() if annotated_frame is not None else None
            if frame is not None:
                with lock:
                    output_frame = cv2.imencode('.jpg', frame)[1].tobytes()
        time.sleep(0.05)


def flask_routes():
    logging.debug("Starting Flask app")
    app.run(host="0.0.0.0", port=5000)


@app.route("/")
def index():
    return render_template("index.html")


@app.route("/video_feed")
def video_feed():
    def generate():
        global output_frame
        logging.debug("Client connected to /video_feed")
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
    streaming = True
    logging.debug("Streaming started")
    return jsonify({"status": "started"})


@app.route("/stop_stream")
def stop_stream():
    global streaming
    streaming = False
    logging.debug("Streaming stopped")
    return jsonify({"status": "stopped"})


@app.route("/detection_status")
def detection_status():
    logging.debug("Status checked: %s", popup_sent)
    return jsonify({"detected": popup_sent})


@app.route("/misca")
def activate():
    logging.debug("/misca route triggered")
    activate_servos()
    return "Servomotor activat"


@app.route("/takeoff")
def takeoff():
    logging.debug("/takeoff route triggered")
    return "Drone Takeoff (dezactivat temporar)"


@app.route("/land")
def land():
    logging.debug("/land route triggered")
    return "Drone Landing (dezactivat temporar)"


if __name__ == "__main__":
    threading.Thread(target=capture_camera, daemon=True, name="CameraThread").start()
    threading.Thread(target=detect_objects, daemon=True, name="DetectionThread").start()
    threading.Thread(target=stream_output, daemon=True, name="StreamThread").start()
    threading.Thread(target=flask_routes, daemon=True, name="FlaskThread").start()
    while True:
        time.sleep(1)
