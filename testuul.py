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
    return render_template_string("""
        <!doctype html>
        <html>
        <head>
            <title>YOLO Stream</title>
            <script>
                function startStream() {
                    fetch('/start_stream');
                }
                function stopStream() {
                    fetch('/stop_stream');
                }
            </script>
        </head>
        <body>
            <h1>YOLO Stream Control</h1>
            <img id="video" src="/video_feed" width="640" height="480">
            <br><br>
            <button onclick="startStream()">Start Stream</button>
            <button onclick="stopStream()">Stop Stream</button>
        </body>
        </html>
    """)

@app.route("/video_feed")
def video_feed():
    def generate():
        while True:
            if not streaming:
                time.sleep(0.1)
                continue
            with lock:
                if output_frame is None:
                    continue
                frame = output_frame
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
