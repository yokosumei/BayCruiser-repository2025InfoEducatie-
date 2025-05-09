from flask import Flask, render_template, Response, jsonify
from ultralytics import YOLO
from picamera2 import Picamera2
import cv2
import threading
import time

app = Flask(__name__)

# Load YOLO model
model = YOLO("my_model.pt")

# Initialize PiCamera2
picam2 = Picamera2()
picam2.configure(picam2.create_preview_configuration(main={"format": 'RGB888', "size": (416, 240)}))  # lower res for FPS
picam2.start()

# Shared state
output_frame = None
lock = threading.Lock()
streaming = False
detection_status = {"detected": False}

def detect_objects():
    global output_frame, streaming, detection_status
    while streaming:
        start = time.time()

        frame = picam2.capture_array()
        results = model(frame, verbose=False)
        result_frame = results[0].plot()

        detection_status["detected"] = any(
            model.names[int(cls)] == "sample" for cls in results[0].boxes.cls
        )

        with lock:
            _, jpeg = cv2.imencode(".jpg", result_frame)
            output_frame = jpeg.tobytes()

        # Control max 30 FPS
        elapsed = time.time() - start
        if elapsed < 1 / 30:
            time.sleep(1 / 30 - elapsed)

@app.route("/")
def index():
    return render_template("index.html")

@app.route("/video_feed")
def video_feed():
    def generate():
        while streaming:
            with lock:
                if output_frame is None:
                    continue
                yield (b"--frame\r\n"
                       b"Content-Type: image/jpeg\r\n\r\n" + output_frame + b"\r\n")
    return Response(generate(), mimetype="multipart/x-mixed-replace; boundary=frame")

@app.route("/start_stream")
def start_stream():
    global streaming
    if not streaming:
        streaming = True
        thread = threading.Thread(target=detect_objects)
        thread.daemon = True
        thread.start()
    return ("", 200)

@app.route("/stop_stream")
def stop_stream():
    global streaming
    streaming = False
    return ("", 200)

@app.route("/detection_status")
def get_detection_status():
    return jsonify(detection_status)

if __name__ == "__main__":
    app.run(host="0.0.0.0", port=5000)
