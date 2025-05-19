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
print(f"[INFO] Output details:")
for i, detail in enumerate(output_details):
    print(f"  {i}: name={detail['name']}, shape={detail['shape']}, dtype={detail['dtype']}")

if len(output_details) != 1:
    print("[EROARE] Modelul nu are un singur tensor de ieșire. Acest script presupune un model YOLOv8 TFLite convertit cu 1 ieșire.")
    sys.exit(1)

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

# === INIȚIALIZARE CAMERĂ ===
picam2 = Picamera2()
picam2.configure(picam2.create_video_configuration(main={"size": (640, 480)}))
picam2.start()
time.sleep(2)

print("[INFO] Pornit cu succes. Se inspectează outputul modelului...")

# === BUCLE PENTRU DEBUG ===
try:
    frame = picam2.capture_array()
    input_tensor = preprocess(frame)
    detections = run_inference(input_tensor)

    print(f"\n=== DEBUG OUTPUT MODEL ===")
    print(f"Shape output detections: {detections.shape}")
    print(f"Exemplu output (primele 5 rânduri):\n{detections[:5]}")

    print("\n[INFO] Ieșire după debug.")
    sys.exit(0)

except KeyboardInterrupt:
    print("\n[INFO] Oprit manual cu Ctrl+C")

finally:
    picam2.stop()
    cv2.destroyAllWindows()
    print("[INFO] Închidere completă.")
