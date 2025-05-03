from flask import Flask, Response
from ultralytics import YOLO
import cv2

app = Flask(__name__)
model = YOLO("my_model.pt")

def gen_frames():
    cap = cv2.VideoCapture(0)
    while True:
        success, frame = cap.read()
        if not success:
            break
        results = model(frame)
        annotated_frame = results[0].plot()
        _, buffer = cv2.imencode('.jpg', annotated_frame)
        frame_bytes = buffer.tobytes()
        yield (b'--frame\r\n'
               b'Content-Type: image/jpeg\r\n\r\n' + frame_bytes + b'\r\n')

@app.route('/video_feed')
def video_feed():
    return Response(gen_frames(), mimetype='multipart/x-mixed-replace; boundary=frame')

@app.route('/')
def index():
    return "<h1>YOLO Stream</h1><img src='/video_feed'>"

if __name__ == '__main__':
    app.run(host='0.0.0.0', port=5000)
