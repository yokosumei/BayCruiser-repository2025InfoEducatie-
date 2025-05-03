from flask import Flask, render_template, Response, jsonify
from ultralytics import YOLO
import cv2

app = Flask(__name__)
model = YOLO('my_model.pt')  # încarcă modelul antrenat

# Flag global pentru detecție
detection_flag = False

# Inițializează camera
camera = cv2.VideoCapture(0)

def generate_frames():
    global detection_flag
    print("[INFO] Generating frames...")
    while True:
        success, frame = camera.read()
        if not success:
            print("[ERROR] Camera read failed.")
            break

        print("[INFO] Frame captured.")
        results = model(frame)
        detections = results.pandas().xyxy[0]
        detection_flag = any(detections['name'] == 'sample')
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
    app.run(host='0.0.0.0', port=5000)
