from flask import Flask, render_template, Response, jsonify
from ultralytics import YOLO
from picamera2 import Picamera2
from gpiozero import AngularServo
import numpy as np
import threading
import cv2
import time

model = YOLO("my_model.pt")
picam2 = Picamera2()
picam2.configure(picam2.create_video_configuration(main={"format": "RGB888", "size": (640, 480)}))
picam2.start()

streaming = False
lock = threading.Lock()
output_frame = None
detected_once = False

app = Flask(__name__)
servo = AngularServo(18, min_pulse_width=0.0006, max_pulse_width=0.0023)

last_detection = {"om_la_inec": False}


def blank_frame():
    img = np.zeros((480, 640, 3), dtype=np.uint8)
    _, buffer = cv2.imencode('.jpg', img)
    return buffer.tobytes()


def detect_objects():
    global output_frame, streaming, last_detection
    while streaming:
        frame = picam2.capture_array()
        results = model(frame, verbose=False)
        annotated = results[0].plot()

        # Detecție "om_la_inec"
        names = results[0].names
        detections = results[0].boxes.cls.tolist()
        detected = any(names[int(cls)] == "om_la_inec" for cls in detections)

        with lock:
            output_frame = cv2.imencode('.jpg', annotated)[1].tobytes()
            last_detection["om_la_inec"] = detected

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


@app.route("/misca")
def activate():
    servo.angle = 110
    time.sleep(2)
    servo.angle = 90
    return "Servomotor mișcat!"


@app.route("/detection_status")
def detection_status():
    global detected_once
    with lock:
        if last_detection["om_la_inec"] and not detected_once:
            detected_once = True
            return jsonify({"detected": True})
        elif not last_detection["om_la_inec"]:
            detected_once = False
    return jsonify({"detected": False})


if __name__ == "__main__":
    app.run(host="0.0.0.0", port=5000)
