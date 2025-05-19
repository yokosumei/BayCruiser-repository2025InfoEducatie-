from flask import Flask, render_template, Response
from picamera2 import Picamera2
import cv2
import numpy as np
import tflite_runtime.interpreter as tflite

# Flask app
app = Flask(__name__)

# Initialize camera
picam2 = Picamera2()
picam2.configure(picam2.create_video_configuration(main={"size": (320, 240)}))
picam2.start()

# Load TFLite model
interpreter = tflite.Interpreter(model_path="my_model_final_int8.tflite")
interpreter.allocate_tensors()

# Get model input/output details
input_details = interpreter.get_input_details()
output_details = interpreter.get_output_details()
input_shape = input_details[0]['shape']

# Preprocess + inference
def run_inference(frame):
    image_resized = cv2.resize(frame, (input_shape[2], input_shape[1]))
    input_data = np.expand_dims(image_resized, axis=0)

    # Detect model type: quantizat sau float
    if input_details[0]['dtype'] == np.float32:
        input_data = input_data.astype(np.float32) / 255.0
    else:
        input_data = input_data.astype(np.uint8)

    interpreter.set_tensor(input_details[0]['index'], input_data)
    interpreter.invoke()
    output_data = interpreter.get_tensor(output_details[0]['index'])
    return output_data

# Generator video
def gen():
    while True:
        frame = picam2.capture_array()
        detections = run_inference(frame)

        for det in detections[0]:
            if det[4] < 0.5:
                continue
            x, y, w, h, conf, cls = det[:6]

            x1 = int(x - w / 2)
            y1 = int(y - h / 2)
            x2 = int(x + w / 2)
            y2 = int(y + h / 2)

            # Draw bounding box
            cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
            cv2.putText(frame, f"{int(cls)} {conf:.2f}", (x1, y1 - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

        # Encode frame to JPEG
        ret, jpeg = cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, 70])
        if not ret:
            continue
        yield (b'--frame\r\n'
               b'Content-Type: image/jpeg\r\n\r\n' + jpeg.tobytes() + b'\r\n\r\n')

# Flask routes
@app.route('/')
def index():
    return '''
    <html>
        <head><title>YOLOv11s TFLite Stream</title></head>
        <body>
            <h1>Live Feed cu Detecție</h1>
            <img src="/video_feed" width="640" height="480">
        </body>
    </html>
    '''

@app.route('/video_feed')
def video_feed():
    return Response(gen(), mimetype='multipart/x-mixed-replace; boundary=frame')

# Run app
if __name__ == '__main__':
    app.run(host='0.0.0.0', port=5000)
