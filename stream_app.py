from flask import Flask, render_template, Response, jsonify
import cv2
import threading
import torch

app = Flask(__name__)

# Load YOLO model (înlocuiește cu calea ta dacă e altceva)
model = torch.hub.load('ultralytics/yolov5', 'custom', path='my_model.pt', force_reload=True)

# Variabile globale
streaming = False
detection_result = None
camera = cv2.VideoCapture(0)
fallback_frame = cv2.imread("test_frame.jpg")

# Check if camera is opened
if not camera.isOpened():
    print("[WARN] Camera failed to open, using fallback frame.")
    camera = None
else:
    print("[INFO] Camera opened successfully.")

def generate_frames():
    global detection_result
    while streaming:
        if camera is not None:
            ret, frame = camera.read()
            if not ret:
                print("[ERROR] Failed to read from camera. Using fallback.")
                frame = fallback_frame
        else:
            frame = fallback_frame

        # Run detection
        results = model(frame)
        detection_result = results.pandas().xyxy[0]

        # Draw bounding boxes
        for _, row in detection_result.iterrows():
            label = row['name']
            conf = row['confidence']
            if conf > 0.5:
                x1, y1, x2, y2 = map(int, [row['xmin'], row['ymin'], row['xmax'], row['ymax']])
                cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
                cv2.putText(frame, f'{label} {conf:.2f}', (x1, y1 - 10),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

        ret, buffer = cv2.imencode('.jpg', frame)
        frame_bytes = buffer.tobytes()
        yield (b'--frame\r\n'
               b'Content-Type: image/jpeg\r\n\r\n' + frame_bytes + b'\r\n')

@app.route('/')
def index():
    return render_template('index.html')

@app.route('/start_stream')
def start_stream():
    global streaming
    streaming = True
    return ('', 204)

@app.route('/stop_stream')
def stop_stream():
    global streaming
    streaming = False
    return ('', 204)

@app.route('/video_feed')
def video_feed():
    return Response(generate_frames(),
                    mimetype='multipart/x-mixed-replace; boundary=frame')

@app.route('/detection_status')
def detection_status():
    global detection_result
    detected = False
    if detection_result is not None:
        detected = any(detection_result['name'] == 'sample')
    return jsonify({'detected': detected})

if __name__ == '__main__':
    app.run(host='0.0.0.0', port=5000)
