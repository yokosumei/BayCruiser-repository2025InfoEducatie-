from flask import Flask, render_template, Response, jsonify
from ultralytics import YOLO
from picamera2 import Picamera2
from gpiozero import AngularServo
import numpy as np
import threading
import cv2
import time

app = Flask(__name__)

model = YOLO("my_model.pt")
picam2 = Picamera2()
picam2.configure(picam2.create_video_configuration(main={"format": "RGB888", "size": (640, 480)}))
picam2.start()

servo = AngularServo(18, min_pulse_width=0.0006, max_pulse_width=0.0023)

streaming = False
lock = threading.Lock()
output_frame = None

detected_flag = False
popup_sent = False
cooldown_active = False

def blank_frame():
    img = np.zeros((480, 640, 3), dtype=np.uint8)
    _, buffer = cv2.imencode('.jpg', img)
    return buffer.tobytes()

def detect_objects():
    global output_frame, streaming, detected_flag, popup_sent, cooldown_active

    while streaming:
        frame = picam2.capture_array()
        results = model(frame, verbose=False)
        annotated = results[0].plot()

        # Detectăm "om_la_inec"
        names = results[0].names
        class_ids = results[0].boxes.cls.tolist()
        detected = any(names[int(cls_id)] == "om_la_inec" for cls_id in class_ids)

        if detected:
            if not detected_flag and not cooldown_active:
                detected_flag = True
                popup_sent = True  
        else:
            detected_flag = False  # resetare

        with lock:
            output_frame = cv2.imencode('.jpg', annotated)[1].tobytes()

        time.sleep(0.05)

@app.route("/")
def index():
    return render_template("index.html")

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
    global popup_sent
    return jsonify({"detected": popup_sent})

@app.route("/misca")
def activate():
    global detected_flag, popup_sent, cooldown_active

    print(" Servomotor activat.")
    servo.angle = 45
    time.sleep(2)
    servo.angle = 0
    detected_flag = False
    popup_sent = False
    cooldown_active = True

    # Cooldown de 10 secunde într-un thread separat
    threading.Thread(target=cooldown_timer).start()

    return "Servomotor activat"

def cooldown_timer():
    global cooldown_active
    print("Cooldown 10s...")
    time.sleep(10)
    cooldown_active = False
    print("Cooldown terminat.")
