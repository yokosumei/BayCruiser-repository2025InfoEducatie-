from flask import Flask, render_template, Response, jsonify
import torch
import cv2

app = Flask(__name__)

# Încarcă modelul yolo11s (custom yolov5s) din fișier local
model = torch.hub.load('ultralytics/yolo11s', 'custom', path='my_model.pt', source='local')

# Flag global pentru detecție
detection_flag = False

# Inițializează camera
camera = cv2.VideoCapture(0)

def generate_frames():
    global detection_flag
    while True:
        success, frame = camera.read()
        if not success:
            break

        results = model(frame)
        detections = results.pandas().xyxy[0]

        # Verificăm dacă obiectul "sample" este detectat
        detection_flag = any(detections['name'] == 'sample')

        # Desenăm pătrate pe frame
        annotated_frame = results.render()[0]
        _, buffer = cv2.imencode('.jpg', annotated_frame)
        frame_bytes = buffer.tobytes()

        yield (b'--frame\r\n'
               b'Content-Type: image/jpeg\r\n\r\n' + frame_bytes + b'\r\n')

@app.route('/')
def index():
    return render_template('index.html')

@app.route('/video_feed')
def video_feed():
    return Response(generate_frames(), mimetype='multipart/x-mixed-replace; boundary=frame')

@app.route('/detection_status')
def detection_status():
    return jsonify({'detected': detection_flag})

if __name__ == '__main__':
    app.run(host='0.0.0.0')
