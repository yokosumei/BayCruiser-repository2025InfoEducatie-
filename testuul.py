from flask import Flask, Response
from ultralytics import YOLO
from picamera2 import Picamera2
import numpy as np
import cv2
import time

app = Flask(__name__)

# Încarcă modelul YOLO (yolo11n)
model = YOLO("yolo11n.pt")  # înlocuiește cu path-ul tău, ex: "models/yolo11n.pt"

# Inițializează camera
picam2 = Picamera2()
picam2.configure(picam2.create_video_configuration(main={"format": "RGB888", "size": (640, 480)}))
picam2.start()

def gen_frames():
    while True:
        frame = picam2.capture_array()
        
        # Detectare cu YOLO
        results = model(frame, verbose=False)
        annotated = results[0].plot()

        # Encodare JPEG
        ret, buffer = cv2.imencode('.jpg', annotated)
        frame_bytes = buffer.tobytes()

        yield (b'--frame\r\n'
               b'Content-Type: image/jpeg\r\n\r\n' + frame_bytes + b'\r\n')
        time.sleep(0.05)  # ~20 FPS

@app.route('/video_feed')
def video_feed():
    return Response(gen_frames(), mimetype='multipart/x-mixed-replace; boundary=frame')

@app.route('/')
def index():
    return "<h1>YOLOv11n cu Picamera2</h1><img src='/video_feed'>"

if __name__ == '__main__':
    app.run(host='0.0.0.0', port=5000)
