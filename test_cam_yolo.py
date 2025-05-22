from flask import Flask, render_template, Response, jsonify
from ultralytics import YOLO
from picamera2 import Picamera2
import RPi.GPIO as GPIO
import numpy as np
import threading
import cv2
import time

app = Flask(__name__)

# Inițializări
model = YOLO("my_model.pt")
picam2 = Picamera2()
picam2.configure(picam2.create_video_configuration(main={"format": "RGB888", "size": (640, 480)}))
picam2.start()

GPIO.setmode(GPIO.BOARD)

GPIO.setup(11,GPIO.OUT)
servo1 = GPIO.PWM(11,50)
GPIO.setup(12,GPIO.OUT)
servo2 = GPIO.PWM(12,50) 
servo1.start(0)
servo2.start(0)

streaming = False
lock = threading.Lock()
output_frame = None

detected_flag = False  # semnal intern: este detectat ACUM în cadru
popup_sent = False     # semnal pentru frontend: trebuie să afișeze popup

def blank_frame():
    img = np.zeros((480, 640, 3), dtype=np.uint8)
    _, buffer = cv2.imencode('.jpg', img)
    return buffer.tobytes()

def detect_objects():
    global output_frame, streaming, detected_flag, popup_sent

    while streaming:
        frame = picam2.capture_array()
        results = model(frame, verbose=False)
        annotated = results[0].plot()

        # Verifică dacă obiectul "om_la_inec" este detectat
        names = results[0].names
        class_ids = results[0].boxes.cls.tolist()
        detected = any(names[int(cls_id)] == "om_la_inec" for cls_id in class_ids)

        if detected:
            if not detected_flag:
                detected_flag = True
                popup_sent = True  # semnalăm frontend-ul că trebuie popup
        else:
            detected_flag = False
            popup_sent = False  # resetăm când dispare din cadru

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
    return jsonify({"detected": popup_sent})

@app.route("/misca")
def activate():
    servo1.ChangeDutyCycle(7)
    servo2.ChangeDutyCycle(7)
    time.sleep(1)
    servo1.ChangeDutyCycle(0)
    servo2.ChangeDutyCycle(0)
    return "Servomotor activat"

if __name__ == "__main__":
    app.run(host="0.0.0.0", port=5000)
