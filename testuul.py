from flask import Flask, Response, render_template_string, jsonify
from ultralytics import YOLO
from picamera2 import Picamera2
import numpy as np
import cv2
import time
import threading

app = Flask(__name__)

model = YOLO("my_model.pt")  # sau "yolo11n.pt"

picam2 = Picamera2()
picam2.configure(picam2.create_video_configuration(main={"format": "RGB888", "size": (640, 480)}))
picam2.start()

streaming = False
lock = threading.Lock()
output_frame = None

# frame negru ca fallback
def blank_frame():
    img = np.zeros((480, 640, 3), dtype=np.uint8)
    _, buffer = cv2.imencode('.jpg', img)
    return buffer.tobytes()

def detect_objects():
    global output_frame, streaming
    while streaming:
        frame = picam2.capture_array()
        results = model(frame, verbose=False)
        annotated = results[0].plot()
        with lock:
            output_frame = cv2.imencode('.jpg', annotated)[1].tobytes()
        time.sleep(0.05)

@app.route("/")
def index():

@app.route("/video_feed")
def video_feed():
    def generate():
        global output_frame
        while True:
            with lock:
                frame = output_frame if streaming and output_frame is not None else blank_frame()
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

if __name__ == '__main__':
    app.run(host='0.0.0.0', port=5000)
