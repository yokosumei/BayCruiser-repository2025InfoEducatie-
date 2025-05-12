from flask import Flask, Response
from ultralytics import YOLO
from picamera2 import Picamera2
import cv2
import numpy as np
import time

app = Flask(__name__)
model = YOLO("my_model.pt")

picam2 = Picamera2()
picam2.configure(picam2.create_video_configuration(main={"size": (640, 480)}))
picam2.start()

def gen_frames():
    while True:
        frame = picam2.capture_array()
        results = model(frame)
        annotated = results[0].plot()

        # Convert annotated image (NumPy array) to JPEG
        ret, buffer = cv2.imencode('.jpg', annotated)
        frame_bytes = buffer.tobytes()

        yield (b'--frame\r\n'
               b'Content-Type: image/jpeg\r\n\r\n' + frame_bytes + b'\r\n')
        time.sleep(0.03)  # ușor delay pentru a reduce încărcarea CPU

@app.route('/video_feed')
def video_feed():
    return Response(gen_frames(), mimetype='multipart/x-mixed-replace; boundary=frame')

@app.route('/')
def index():
    return "<h1>YOLO Stream</h1><img src='/video_feed'>"

if __name__ == '__main__':
    app.run(host='0.0.0.0', port=5000)
