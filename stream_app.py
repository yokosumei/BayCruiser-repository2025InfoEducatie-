from flask import Flask, render_template, Response, jsonify
from ultralytics import YOLO
import cv2

app = Flask(__name__)

# Încarcă modelul și clasele
model = YOLO('my_model.pt')
class_names = model.names

# Inițializează camera
camera = cv2.VideoCapture(0)
camera.set(cv2.CAP_PROP_FRAME_WIDTH, 320)
camera.set(cv2.CAP_PROP_FRAME_HEIGHT, 240)

# Flag global pentru streaming și detecție
streaming = True
detection_flag = False

def generate_frames():
    global streaming, detection_flag

    frame_count = 0

    while streaming:
        success, frame = camera.read()
        if not success:
            break

        frame_count += 1
        if frame_count % 2 != 0:
            continue  # Skip un frame din 2 pentru performanță

        rgb_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        results = model(rgb_frame)

        boxes = results[0].boxes
        detections = boxes.xyxy.cpu().numpy()
        scores = boxes.conf.cpu().numpy()
        classes = boxes.cls.cpu().numpy()

        detection_flag = False

        for box, score, cls_id in zip(detections, scores, classes):
            x1, y1, x2, y2 = map(int, box[:4])
            label = class_names[int(cls_id)]
            confidence = float(score)

            if confidence > 0.5:
                cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
                cv2.putText(frame, f'{label} {confidence:.2f}', (x1, y1 - 10),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

                if label == 'sample':
                    detection_flag = True

        ret, buffer = cv2.imencode('.jpg', frame)
        frame = buffer.tobytes()

        yield (b'--frame\r\n'
               b'Content-Type: image/jpeg\r\n\r\n' + frame + b'\r\n')


@app.route('/')
def index():
    return render_template('index.html')


@app.route('/video_feed')
def video_feed():
    return Response(generate_frames(), mimetype='multipart/x-mixed-replace; boundary=frame')


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
