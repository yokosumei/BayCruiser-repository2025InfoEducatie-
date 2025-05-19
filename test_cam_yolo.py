import cv2
import numpy as np
import tflite_runtime.interpreter as tflite
from picamera2 import Picamera2
import time

# === Configurări ===
MODEL_PATH = "model.tflite"
INPUT_SIZE = (640, 640)
CLASSES = ["fara_inec", "om_la_inec"]
CONFIDENCE_THRESHOLD = 0.5

# === Inițializare cameră ===
picam2 = Picamera2()
picam2.preview_configuration.main.size = INPUT_SIZE
picam2.preview_configuration.main.format = "RGB888"
picam2.configure("preview")
picam2.start()

# === Încărcare model TFLite ===
interpreter = tflite.Interpreter(model_path=MODEL_PATH)
interpreter.allocate_tensors()
input_details = interpreter.get_input_details()
output_details = interpreter.get_output_details()

# === Funcție preprocesare imagine ===
def preprocess(frame):
    resized = cv2.resize(frame, INPUT_SIZE)
    normalized = resized / 255.0
    input_tensor = np.expand_dims(normalized.astype(np.float32), axis=0)
    return input_tensor

# === Funcție rulare inferență ===
def run_inference(tensor):
    interpreter.set_tensor(input_details[0]['index'], tensor)
    interpreter.invoke()
    output_data = interpreter.get_tensor(output_details[0]['index'])[0]  # shape: (6, 8400)
    output_data = np.transpose(output_data)  # shape: (8400, 6)
    return output_data

# === Funcție desenare detecții ===
def draw_detections(frame, detections):
    h, w, _ = frame.shape
    for det in detections:
        x, y, width, height, score0, score1 = det
        conf = max(score0, score1)
        if conf > CONFIDENCE_THRESHOLD:
            cls = 0 if score0 > score1 else 1

            x1 = int((x - width / 2) * w)
            y1 = int((y - height / 2) * h)
            x2 = int((x + width / 2) * w)
            y2 = int((y + height / 2) * h)

            label = f"{CLASSES[cls]}: {conf:.2f}"
            color = (0, 255, 0) if cls == 0 else (0, 0, 255)

            cv2.rectangle(frame, (x1, y1), (x2, y2), color, 2)
            cv2.putText(frame, label, (x1, y1 - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)

# === Main loop ===
try:
    while True:
        frame = picam2.capture_array()
        input_tensor = preprocess(frame)
        detections = run_inference(input_tensor)
        filtered = [d for d in detections if max(d[4], d[5]) > CONFIDENCE_THRESHOLD]

        draw_detections(frame, filtered)
        cv2.imshow("Detecție YOLOv11s (TFLite)", frame)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

except KeyboardInterrupt:
    print("[INFO] Întrerupt de utilizator.")

finally:
    cv2.destroyAllWindows()
    picam2.stop()
