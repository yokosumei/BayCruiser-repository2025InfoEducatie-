from flask import Flask, render_template, Response, request
import cv2
import torch
import threading
import time

app = Flask(__name__)

# Încarcă modelul YOLO o singură dată
model = torch.hub.load('ultralytics/yolo11s', 'custom', path='my_model.pt')
streaming = False
stream_lock = threading.Lock()


def generate_frames():
    global streaming
    cap = cv2.VideoCapture(0)  # camera index (0 = default USB cam)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 320)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 240)
    cap.set(cv2.CAP_PROP_FPS, 15)

    while True:
        with stream_lock:
            if not streaming:
                break

        ret, frame = cap.read()
        if not ret:
            break

        # det speedboost
        small_frame = cv2.resize(frame, (320, 240))

        # YOLO det
        results = model(small_frame, size=320)
        annotated = results.render()[0]

        # JPEG encoding
        ret, buffer = cv2.imencode('.jpg', annotated)
        frame = buffer.tobytes()

        # frame-->browser
        yield (b'--frame\r\n'
               b'Content-Type: image/jpeg\r\n\r\n' + frame + b'\r\n')
        time.sleep(0.03)  # 30 FPS max

    cap.release()


@app.route('/video_feed')
def video_feed():
    return Response(generate_frames(), mimetype='multipart/x-mixed-replace; boundary=frame')


@app.route('/start_stream')
def start_stream():
    global streaming
    with stream_lock:
        streaming = True
    return '', 204


@app.route('/stop_stream')
def stop_stream():
    global streaming
    with stream_lock:
        streaming = False
    return '', 204


if __name__ == '__main__':
    app.run(host='0.0.0.0', port=5000, threaded=True)
