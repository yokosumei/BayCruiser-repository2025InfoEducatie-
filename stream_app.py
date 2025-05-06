from flask import Flask, render_template, Response, jsonify
from ultralytics import YOLO
import cv2
import threading
import time

app = Flask(__name__)
model = YOLO("my_model.pt")  # modelul tău YOLO
class_names = model.names

streaming = False
detection_flag = False
last_frame = None
frame_lock = threading.Lock()
stream_thread = None

def process_stream():
    global last_frame, detection_flag, streaming

    cap = cv2.VideoCapture(0)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
    cap.set(cv2.CAP_PROP_FPS, 30)

    while streaming:
        success, frame = cap.read()
        if not success:
            continue

        results = model(frame, verbose=False)
        detection_flag = False

        boxes = results[0].boxes
        if boxes is not None and boxes.cls is not None:
            scores = boxes.conf.cpu().numpy()
            classes = boxes.cls.cpu().numpy()

            for score, cls_id in zip(scores, classes):
                label = class_names[int(cls_id)]
                if score > 0.5 and label == "sample":
                    detection_flag = True
                    print(">>> SAMPLE DETECTAT <<<")
                    break

        annotated = results[0].plot()
        ret, buffer = cv2.imencode('.jpg', annotated)
        if ret:
            with frame_lock:
                last_frame = buffer.tobytes()

        time.sleep(1 / 30.0)

    cap.release()
    with frame_lock:
        last_frame = None
    detection_flag = False

@app.route('/')
def index():
    return render_template('index.html')

@app.route('/video_feed')
def video_feed():
    def generate():
        while streaming:
            with frame_lock:
                if last_frame is not None:
                    yield (b'--frame\r\n'
                           b'Content-Type: image/jpeg\r\n\r\n' + last_frame + b'\r\n')
            time.sleep(1 / 30.0)
    return Response(generate(), mimetype='multipart/x-mixed-replace; boundary=frame')

@app.route('/start_stream')
def start_stream():
    global streaming, stream_thread
    if not streaming:
        streaming = True
        stream_thread = threading.Thread(target=process_stream, daemon=True)
        stream_thread.start()
    return '', 200

@app.route('/stop_stream')
def stop_stream():
    global streaming
    streaming = False
    return '', 200

@app.route('/detection_status')
def detection_status():
    return jsonify({'detected': detection_flag})

if __name__ == '__main__':
    app.run(host='0.0.0.0', port=5000)
