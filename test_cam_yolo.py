from flask import Flask, render_template, Response
import cv2
import numpy as np
import tflite_runtime.interpreter as tflite
from picamera2 import Picamera2
import time
import os

app = Flask(__name__)

# === Încarcă modelul TFLite ===
model_path = "weights/my_model_final_int8.tflite"

if not os.path.exists(model_path):
    raise FileNotFoundError(f"Modelul nu există: {model_path}")

interpreter = tflite.Interpreter(model_path=model_path)
interpreter.allocate_tensors()

input_details = interpreter.get_input_details()
output_details = interpreter.get_output_details()
_, input_height, input_width, _ = input_details[0]['shape']
input_dtype = input_details[0]['dtype']
print(f"Modelul așteaptă input de tip: {input_dtype}")

# Etichete
CLASSES = ["om_la_inec", "inotator"]
CONFIDENCE_THRESHOLD = 0.5

# === Inițializează camera ===
picam2 = Picamera2()
picam2.configure(picam2.create_video_configuration(main={"size": (640, 480)}))
picam2.start()

def preprocess_frame(frame):
    # Elimină canalul alfa dacă există (RGBA -> RGB)
    if frame.shape[2] == 4:
        frame = cv2.cvtColor(frame, cv2.COLOR_RGBA2RGB)

    input_image = cv2.resize(frame, (input_width, input_height))

    if input_dtype == np.float32:
        input_tensor = np.expand_dims(input_image, axis=0).astype(np.float32)
        input_tensor /= 255.0  # normalize to [0, 1]
    elif input_dtype == np.uint8:
        input_tensor = np.expand_dims(input_image, axis=0).astype(np.uint8)
    else:
        raise ValueError(f"Tip de input nesuportat: {input_dtype}")

    return input_tensor

def run_inference(input_tensor):
    interpreter.set_tensor(input_details[0]['index'], input_tensor)
    interpreter.invoke()

    boxes = interpreter.get_tensor(output_details[0]['index'])[0]
    classes = interpreter.get_tensor(output_details[1]['index'])[0]
    scores = interpreter.get_tensor(output_details[2]['index'])[0]
    return boxes, classes, scores

def draw_detections(frame, boxes, classes, scores):
    h, w, _ = frame.shape
    for i in range(len(scores)):
        if scores[i] > CONFIDENCE_THRESHOLD:
            ymin, xmin, ymax, xmax = boxes[i]
            (left, top, right, bottom) = (
                int(xmin * w), int(ymin * h),
                int(xmax * w), int(ymax * h)
            )
            label = f"{CLASSES[int(classes[i])]}: {scores[i]:.2f}"
            cv2.rectangle(frame, (left, top), (right, bottom), (0, 255, 0), 2)
            cv2.putText(frame, label, (left, top - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

@app.route('/')
def index():
    return render_template('index.html')

@app.route('/video_feed')
def video_feed():
    return Response(generate_frames(),
                    mimetype='multipart/x-mixed-replace; boundary=frame')

def generate_frames():
    while True:
        frame = picam2.capture_array()
        input_tensor = preprocess_frame(frame)
        boxes, classes, scores = run_inference(input_tensor)
        draw_detections(frame, boxes, classes, scores)

        _, buffer = cv2.imencode('.jpg', frame)
        frame = buffer.tobytes()

        yield (b'--frame\r\n'
               b'Content-Type: image/jpeg\r\n\r\n' + frame + b'\r\n')

if __name__ == '__main__':
    app.run(host='0.0.0.0', port=5000)

cv2.destroyAllWindows()
picam2.stop()
