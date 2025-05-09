from flask import Flask, render_template, Response, jsonify
from ultralytics import YOLO
from picamera2 import Picamera2
import cv2
import threading
import time

app = Flask(__name__)

model = YOLO("my_model.pt")

picam2 = Picamera2()
picam2.configure(picam2.create_preview_configuration(main={"format": 'RGB888', "size": (640, 480)}))
picam2.start()

output_frame = None
lock = threading.Lock()
streaming = False
detection_status = {"detected": False}

def detect_objects():
    global output_frame, streaming, detection_status
    while streaming:
        frame = picam2.capture_array()

        # Convertește frame-ul la 3 canale (evită eroarea RGBA)
        if frame.shape[2] == 4:
            frame = cv2.cvtColor(frame, cv2.COLOR_RGBA2RGB)

        results = model(frame, verbose=False)
        result_frame = results[0].plot()

        detection_status["detected"] = any(
            model.names[int(cls)] == "sample" for cls in results[0].boxes.cls
        )

        with lock:
            output_frame = cv2.cvtColor(result_frame, cv2.COLOR_RGB2BGR)

        time.sleep(0.03)

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
                ret, buffer = cv2.imencode('.jpg', output_frame)
                frame = buffer.tobytes()
            yield (b"--frame\r\n"
                   b"Content-Type: image/jpeg\r\n\r\n" + frame + b"\r\n")
            time.sleep(0.03)
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
