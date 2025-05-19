import cv2
import numpy as np
import tflite_runtime.interpreter as tflite
from picamera2 import Picamera2
import time

# === CONFIG ===
MODEL_PATH = "weights/my_model_final_int8.tflite"
SCORE_THRESHOLD = 0.5

# === INIT TFLITE MODEL ===
interpreter = tflite.Interpreter(model_path=MODEL_PATH)
interpreter.allocate_tensors()
input_details = interpreter.get_input_details()
output_details = interpreter.get_output_details()

# Model input shape (ex: [1, 224, 224, 3])
input_shape = input_details[0]['shape']
model_height, model_width = input_shape[1], input_shape[2]

# === INIT CAMERA ===
picam2 = Picamera2()
picam2.preview_configuration.main.size = (model_width, model_height)
picam2.preview_configuration.main.format = "RGB888"
picam2.configure("preview")
picam2.start()
time.sleep(1)

# === PREPROCESS FUNCTION ===
def preprocess(image):
    resized = cv2.resize(image, (model_width, model_height))
    input_data = np.expand_dims(resized, axis=0).astype(np.uint8)
    return input_data

# === MAIN LOOP ===
while True:
    frame = picam2.capture_array()
    input_data = preprocess(frame)

    interpreter.set_tensor(input_details[0]['index'], input_data)
    interpreter.invoke()

    # Assume output shape: [1, num_detections, 6] -> [x1, y1, x2, y2, score, class_id]
    output_data = interpreter.get_tensor(output_details[0]['index'])[0]

    for detection in output_data:
        x1, y1, x2, y2, score, class_id = detection
        if score < SCORE_THRESHOLD:
            continue

        # Scale bbox to frame size
        x1 = int(x1 * frame.shape[1])
        y1 = int(y1 * frame.shape[0])
        x2 = int(x2
