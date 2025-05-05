from flask import Flask, render_template, Response, jsonify
from ultralytics import YOLO
import cv2

app = Flask(__name__)
model = YOLO("my_model.pt")

# Global flags
streaming = True
detection_flag = False

camera = cv2.VideoCapture(0)
camera.set(cv2.CAP_PROP_FRAME_WIDTH, 320)
camera.set(cv2.CAP_PROP_FRAME_HEIGHT, 240)

class_names = model.names

def gen_frames():
    global detection_flag, streaming

    while streaming:
        success, frame = camera.read()
        if not success:
            break

        results = model(frame, verbose=False)
        detection_flag = False

        boxes = results[0].boxes
        scores = boxes.conf.cpu().numpy()
        classes = boxes.cls.cpu().numpy()

        # verificăm dacă există clasa "sample"
        for score, cls_id in zip(scores, classes):
            label = class_names[int(cls_id)]
            if score > 0.5 and label == "sample":
                detection_flag = True
                print(">>> SAMPLE DETECTAT <<<")

        annotated_frame = results[0].plot()
        _, buffer = cv2.imencode('.jpg', annotated_frame)
        frame_bytes = buffer.tobytes()

        yield (b'--frame\r\n'
               b'Content-Type: image/jpeg\r\n\r\n' + frame_bytes + b'\r\n')


@app.route('/')
def index():
    return render_template('index.html')


@app.route('/video_feed')
def video_feed():
    return Response(gen_frames(), mimetype='multipart/x-mixed-replace; boundary=frame')


@app.route('/start_stream')
def start_stream():
    global streaming
    streaming = True
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
