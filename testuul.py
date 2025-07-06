import cv2
import numpy as np
import onnxruntime as ort
from picamera2 import Picamera2

# === 1. Inițializează camera ===
picam2 = Picamera2()
picam2.preview_configuration.main.size = (640, 640)  # input pt model ONNX
picam2.preview_configuration.main.format = "RGB888"
picam2.preview_configuration.align()
picam2.configure("preview")
picam2.start()

# === 2. Încarcă modelul ONNX ===
session = ort.InferenceSession("yolo11n-pose.onnx", providers=["CPUExecutionProvider"])
input_name = session.get_inputs()[0].name

# === 3. Funcție pentru preprocesare imagine ===
def preprocess(image):
    resized = cv2.resize(image, (640, 640))
    img = resized.astype(np.float32) / 255.0
    img = np.transpose(img, (2, 0, 1))  # HWC -> CHW
    img = np.expand_dims(img, axis=0)  # [1, 3, 640, 640]
    return img

# === 4. Loop principal ===
while True:
    frame = picam2.capture_array()
    input_tensor = preprocess(frame)

    # === 5. Inferență ONNX ===
    outputs = session.run(None, {input_name: input_tensor})

    # === 6. TODO: decodificare keypoints + desenare ===
    # outputs[0] are shape [1, 56, 8400] -> trebuie decodificat manual
    # pentru acum doar afișăm imaginea inițială

    cv2.imshow("Camera", frame)

    if cv2.waitKey(1) == ord("q"):
        break

cv2.destroyAllWindows()
