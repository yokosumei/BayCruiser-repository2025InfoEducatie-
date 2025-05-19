import cv2
import numpy as np
import tflite_runtime.interpreter as tflite
from picamera2 import Picamera2
import time
import os

# === Setări model ===
MODEL_PATH = "weights/my_model_final_int8.tflite"
CLASSES = ["om_la_inec", "inotator"]
CONFIDENCE_THRESHOLD = 0.5

# === Încarcă modelul TFLite ===
if not os.path.exists(MODEL_PATH):
    raise FileNotFoundError(f"[EROARE] Modelul nu există: {MODEL_PATH}")

interpreter = tflite.Interpreter(model_path=MODEL_PATH)
interpreter.allocate_tensors()

input_details = interpreter.get_input_details()
output_details = interpreter.get_output_details()

# Debug info despre model
print("[DEBUG] Detalii input:")
print(input_details)
print("[DEBUG] Detalii output:")
for i, detail in enumerate(output_details):
    print(f"{i}: name={detail['name']}, shape={detail['shape']}, dtype={detail['dtype']}")

_, input_height, input_width, _ = input_details[0]['shape']
input_dtype = input_details[0]['dtype']
print(f"[INFO] Modelul așteaptă input de tip: {input_dtype}, dimensiune: {input_width}x{input_height}")

# === Inițializează camera ===
picam2 = Picamera2()
picam2.configure(picam2.create_video_configuration(main={"size": (640, 480)}))
picam2.start()
time.sleep(1)

# === Preprocesare ===
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

# === Rulare inferență compatibilă cu YOLOv11s în TFLite (1 ieșire: [1,6,8400]) ===
def run_inference(input_tensor):
    interpreter.set_tensor(input_details[0]['index'], input_tensor)
    interpreter.invoke()

    output_data = interpreter.get_tensor(output_details[0]['index'])[0]  # shape: (6, 8400)
    output_data = np.transpose(output_data)  # shape: (8400, 6)

    boxes = []
    classes = []
    scores = []

    for det in output_data:
        x, y, w, h, conf, cls = det
        if conf > CONFIDENCE_THRESHOLD:
            xmin = x - w / 2
            ymin = y - h / 2
            xmax = x + w / 2
            ymax = y + h / 2

            boxes.append([ymin, xmin, ymax, xmax])
            classes.append(int(cls))
            scores.append(conf)

    return np.array(boxes), np.array(classes), np.array(scores)

# === Desenează detecțiile pe cadru ===
def draw_detections(frame, boxes, classes, scores):
    h, w, _ = frame.shape
    for i in range(len(scores)):
        ymin, xmin, ymax, xmax = boxes[i]
        (left, top, right, bottom) = (
            int(xmin * w), int(ymin * h),
            int(xmax * w), int(ymax * h)
        )
        label = f"{CLASSES[classes[i]]}: {scores[i]:.2f}"
        cv2.rectangle(frame, (left, top), (right, bottom), (0, 255, 0), 2)
        cv2.putText(frame, label, (left, top - 10),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

# === Loop principal pentru testare ===
try:
    while True:
        frame = picam2.capture_array()
        input_tensor = preprocess_frame(frame)
        boxes, classes, scores = run_inference(input_tensor)
        draw_detections(frame, boxes, classes, scores)

        # Afișare imagine
        cv2.imshow("Detectii", frame)

        # Ieșire cu 'q'
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

except KeyboardInterrupt:
    print("[INFO] Oprit de utilizator.")

finally:
    cv2.destroyAllWindows()
    picam2.stop()
