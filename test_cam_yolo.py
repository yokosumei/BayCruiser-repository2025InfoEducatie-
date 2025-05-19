import cv2
import numpy as np
import tflite_runtime.interpreter as tflite
from picamera2 import Picamera2
import time
import os
import sys

# === CONFIG ===
MODEL_PATH = "weights/my_model_final_int8.tflite"
CLASSES = ["om_la_inec", "inotator"]
CONFIDENCE_THRESHOLD = 0.5

# === VERIFICARE EXISTENȚĂ MODEL ===
if not os.path.exists(MODEL_PATH):
    print(f"[EROARE] Modelul nu există la: {MODEL_PATH}")
    sys.exit(1)

# === ÎNCARCĂ MODELUL TFLITE ===
interpreter = tflite.Interpreter(model_path=MODEL_PATH)
interpreter.allocate_tensors()

input_details = interpreter.get_input_details()
output_details = interpreter.get_output_details()

input_shape = input_details[0]['shape']
input_dtype = input_details[0]['dtype']
_, input_height, input_width, _ = input_shape

print(f"[INFO] Input shape: {input_shape}, dtype: {input_dtype}")
print(f"[INFO] Output shape: {output_details[0]['shape']}")

# === FUNCȚII ===
def preprocess(frame):
    if frame.shape[2] == 4:
        frame = cv2.cvtColor(frame, cv2.COLOR_RGBA2RGB)
    image = cv2.resize(frame, (input_width, input_height))
    if input_dtype == np.float32:
        tensor = np.expand_dims(image, axis=0).astype(np.float32) / 255.0
    elif input_dtype == np.uint8:
        tensor = np.expand_dims(image, axis=0).astype(np.uint8)
    else:
        raise ValueError(f"Tip de input nesuportat: {input_dtype}")
    return tensor

def run_inference(tensor):
    interpreter.set_tensor(input_details[0]['index'], tensor)
    interpreter.invoke()
    output_data = interpreter.get_tensor(output_details[0]['index'])[0]
    return output_data

def postprocess(output_data, original_shape):
    h, w, _ = original_shape
    predictions = []

    output_data = output_data.T  # (8400, 6)
    for det in output_data:
        x, y, w_box, h_box, conf, cls_id = det
        if conf > CONFIDENCE_THRESHOLD:
            x1 = (x - w_box / 2)
            y1 = (y - h_box / 2)
            x2 = (x + w_box / 2)
            y2 = (y + h_box / 2)

            # Normalize la dimensiunea originală
            predictions.append([
                x1, y1, x2, y2,
                conf, cls_id
            ])
    return predictions

def draw_detections(frame, predictions):
    h, w, _ = frame.shape
    for det in predictions:
        x1, y1, x2, y2, conf, cls = det
        left = int(x1 * w)
        top = int(y1 * h)
        right = int(x2 * w)
        bottom = int(y2 * h)
        class_id = int(cls)
        label = f"{CLASSES[class_id]}: {conf:.2f}"

        cv2.rectangle(frame, (left, top), (right, bottom), (0, 255, 0), 2)
        cv2.putText(frame, label, (left, top - 10),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

# === INIȚIALIZARE CAMERĂ ===
picam2 = Picamera2()
picam2.configure(picam2.create_video_configuration(main={"size": (640, 480)}))
picam2.start()
time.sleep(2)

print("[INFO] Pornit cu succes. Apasă Q pentru a ieși.")

# === BUCLE PRINCIPAL ===
try:
    while True:
        frame = picam2.capture_array()
        input_tensor = preprocess(frame)
        raw_output = run_inference(input_tensor)
        predictions = postprocess(raw_output, frame.shape)
        draw_detections(frame, predictions)

        cv2.imshow("YOLOv11s TFLite Live", frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

except KeyboardInterrupt:
    print("\n[INFO] Oprit manual cu Ctrl+C")

finally:
    picam2.stop()
    cv2.destroyAllWindows()
    print("[INFO] Închidere completă.")
