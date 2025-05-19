import cv2
import numpy as np
import os
import tflite_runtime.interpreter as tflite
from picamera2 import Picamera2

# === Config ===
model_path = "weights/my_model_final_int8.tflite"
CLASSES = ["om_la_inec", "inotator"]
CONFIDENCE_THRESHOLD = 0.5

# === Verifică existența modelului ===
if not os.path.exists(model_path):
    raise FileNotFoundError(f"Modelul nu există: {model_path}")

# === Încarcă modelul TFLite ===
interpreter = tflite.Interpreter(model_path=model_path)
interpreter.allocate_tensors()

input_details = interpreter.get_input_details()
output_details = interpreter.get_output_details()

_, input_height, input_width, _ = input_details[0]['shape']
input_dtype = input_details[0]['dtype']

# === DEBUG: Arată output_details ===
print("\n[DEBUG] Detalii output model:")
for i, d in enumerate(output_details):
    print(f"  {i}: name={d['name']}, shape={d['shape']}, dtype={d['dtype']}")
print()

# === Inițializează camera ===
picam2 = Picamera2()
picam2.configure(picam2.create_video_configuration(main={"size": (640, 480)}))
picam2.start()

# === Preprocesare imagine ===
def preprocess_frame(frame):
    if frame.shape[2] == 4:
        frame = cv2.cvtColor(frame, cv2.COLOR_RGBA2RGB)
    input_image = cv2.resize(frame, (input_width, input_height))

    if input_dtype == np.float32:
        input_tensor = np.expand_dims(input_image, axis=0).astype(np.float32)
        input_tensor /= 255.0
    elif input_dtype == np.uint8:
        input_tensor = np.expand_dims(input_image, axis=0).astype(np.uint8)
    else:
        raise ValueError(f"Tip de input nesuportat: {input_dtype}")

    return input_tensor

# === Inference ===
def run_inference(input_tensor):
    interpreter.set_tensor(input_details[0]['index'], input_tensor)
    interpreter.invoke()

    print("[DEBUG] Rulare inferență...")

    # Încearcă să extragi cele 3 ieșiri
    try:
        boxes = interpreter.get_tensor(output_details[0]['index'])[0]
        classes = interpreter.get_tensor(output_details[1]['index'])[0]
        scores = interpreter.get_tensor(output_details[2]['index'])[0]
        return boxes, classes, scores
    except IndexError:
        print("[EROARE] output_details nu are 3 ieșiri. Verifică ordinea sau structura.")
        return None, None, None

# === Desenează detecțiile ===
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

# === Main ===
frame = picam2.capture_array()
input_tensor = preprocess_frame(frame)
boxes, classes, scores = run_inference(input_tensor)

if boxes is not None:
    draw_detections(frame, boxes, classes, scores)
    cv2.imshow("Detecție", frame)
    print("[INFO] Apasă orice tastă pentru a închide fereastra.")
    cv2.waitKey(0)

cv2.destroyAllWindows()
picam2.stop()
