from flask import Flask, Response
from ultralytics import YOLO
from picamera2 import Picamera2
import cv2
import numpy as np
import time

app = Flask(__name__)
model = YOLO("my_model.pt")

# Inițializăm camera
picam2 = Picamera2()
picam2.preview_configuration.main.size = (640, 480)
picam2.preview_configuration.main.format = "RGB888"
picam2.preview_configuration.controls.FrameRate = 20
picam2.configure("preview")
picam2.start()

def gen_frames():
    while True:
        frame = picam2.capture_array()

        # Procesare cu YOLO
        results = model(frame)
        annotated_frame = results[0].plot()

        # Conversie JPEG
        _, buffer = cv2.imencode('.jpg', annotated_frame)
        frame_bytes = buffer.tobytes()

        # Livrare către browser
        yield (b'--frame\r\n'
               b'Content-Type: image/jpeg\r\n\r\n' + frame_bytes + b'\r\n')

        time.sleep(0.05)  # ~20 FPS

@app.route('/video_feed')
def video_feed():
    return Response(gen_frames(), mimetype='multipart/x-mixed-replace; boundary=frame')

@app.route('/')
def index():
    return "<h1>YOLO Stream cu Camera AI</h1><img src='/video_feed'>"

if __name__ == '__main__':
    app.run(host='0.0.0.0', port=5000)
