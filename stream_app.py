from flask import Flask, render_template, Response, jsonify
from ultralytics import YOLO
import cv2
import threading

app = Flask(__name__)
model = YOLO("my_model.pt")

streaming = False
detection_flag = False
camera = None
frame_lock = threading.Lock()
last_frame = None

class_names = model.names

def gen_frames():
    global detection_flag, streaming, camera, last_frame

    camera = cv2.VideoCapture(0)
    camera.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    camera.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

    while streaming:
        success, frame = camera.read()
        if not success:
            break

        results = model(frame, verbose=False)
        detection_flag = False

        boxes = results[0].boxes
        scores = boxes.conf.cpu().numpy()
        classes = boxes.cls.cpu().numpy()

        for score, cls_id in zip(scores, classes):
            label = class_names[int(cls_id)]
            if score > 0.5 and label == "sample":
                detection_flag = True
                print(">>> SAMPLE DETECTAT <<<")

        annotated_frame = results[0].plot()
        _, buffer = cv2.imencode('.jpg', annotated_frame)
        frame_bytes = buffer.tobytes()

        with frame_lock:
            last_frame = frame_bytes

    camera.release()

@app.route('/')
def index():
    return render_template('index.html')

@app.route('/video_feed')
def video_feed():
    def generate():
        global last_frame
        while streaming:
            with frame_lock:
                if last_frame is not None:
                    yield (b'--frame\r\n'
                           b'Content-Type: image/jpeg\r\n\r\n' + last_frame + b'\r\n')
    return Response(generate(), mimetype='multipart/x-mixed-replace; boundary=frame')

@app.route('/start_stream')
def start_stream():
    global streaming
    if not streaming:
        streaming = True
        thread = threading.Thread(target=gen_frames)
        thread.start()
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
