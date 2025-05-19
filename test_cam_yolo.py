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

def draw_detections(frame, detections):
    h, w, _ = frame.shape
    for det in detections:
        x, y, width, height, score0, score1 = det
        conf = max(score0, score1)
        if conf > CONFIDENCE_THRESHOLD:
            cls = 0 if score0 > score1 else 1

            # Convert YOLO center format to corners
            x1 = int((x - width / 2) * w)
            y1 = int((y - height / 2) * h)
            x2 = int((x + width / 2) * w)
            y2 = int((y + height / 2) * h)

            label = f"{CLASSES[cls]}: {conf:.2f}"
            color = (0, 255, 0) if cls == 0 else (255, 0, 0)

            cv2.rectangle(frame, (x1, y1), (x2, y2), color, 2)
            cv2.putText(frame, label, (x1, y1 - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)

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
        output_data = run_inference(input_tensor)

        # Transpune pentru a avea detecțiile ca (8400, 6)
        output_data = output_data.T

        # Filtrare doar pe cele cu scor decent
        filtered = [det for det in output_data if max(det[4], det[5]) > CONFIDENCE_THRESHOLD]

        draw_detections(frame, filtered)
        cv2.imshow("Detecție YOLOv11s (TFLite)", frame)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

except KeyboardInterrupt:
    print("\n[INFO] Oprit manual cu Ctrl+C")

finally:
    picam2.stop()
    cv2.destroyAllWindows()
    print("[INFO] Închidere completă.")
