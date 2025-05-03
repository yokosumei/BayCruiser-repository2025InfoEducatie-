from flask import Flask, render_template, Response, request, jsonify
import cv2
import torch
import threading
import time

app = Flask(__name__)

# Încarcă modelul YOLO11s
model = torch.hub.load('ultralytics/yolov5', 'custom', path='my_model.pt', source='local')
streaming = False
stream_lock = threading.Lock()
object_detected = False  # pentru alertă în frontend

@app.route('/')
def index():
    return render_template('index.html')

def generate_frames():
    global streaming, object_detected
    cap = cv2.VideoCapture(0)
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

        results = model(frame)
        annotated = results.render()[0]

        # Verifică dacă s-a detectat obiectul „sample”
        labels = results.names
        detected_classes = results.pred[0][:, -1].tolist()
        names = [labels[int(cls)] for cls in detected_classes]
        if "sample" in names:
            object_detected = True
        else:
            object_detected = False

        ret, buffer = cv2.imencode('.jpg', annotated)
        frame = buffer.tobytes()

        yield (b'--frame\r\n'
               b'Content-Type: image/jpeg\r\n\r\n' + frame + b'\r\n')
        time.sleep(0.03)

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

@app.route('/detect_status')
def detect_status():
    return jsonify({'detected': object_detected})

if __name__ == '__main__':
    app.run(host='0.0.0.0', port=5000, threaded=True)
