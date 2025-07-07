import cv2
import numpy as np
import onnxruntime as ort
from picamera2 import Picamera2

# === Setări cameră ===
picam2 = Picamera2()
picam2.preview_configuration.main.size = (640, 640)
picam2.preview_configuration.main.format = "RGB888"
picam2.preview_configuration.align()
picam2.configure("preview")
picam2.start()

# === Încarcă modelul ONNX ===
onnx_path = "yolo11n-pose.onnx"
session = ort.InferenceSession(onnx_path)
input_name = session.get_inputs()[0].name
input_size = 640

while True:
    # === Capturează un frame din cameră ===
    frame = picam2.capture_array()
    original_h, original_w = frame.shape[:2]

    # === Preprocesare pentru model ===
    img = cv2.resize(frame, (input_size, input_size))
    img = img.astype(np.float32) / 255.0
    img = img.transpose(2, 0, 1)  # Channel-first
    img = np.expand_dims(img, axis=0)

    # === Inferență ===
    outputs = session.run(None, {input_name: img})
    output = outputs[0][0]  # (56, 8400)

    # === Extrage keypoints ===
    keypoints = output[:51, :]  # 17 * 3 = 51
    keypoints = keypoints.reshape(17, 3, -1)
    keypoints = keypoints.mean(axis=2)  # (17, 3)

    # === Desenare keypoints ===
    for i in range(17):
        x, y, conf = keypoints[i]
        if conf > 0.5:
            cx = int(x / input_size * original_w)
            cy = int(y / input_size * original_h)
            cv2.circle(frame, (cx, cy), 3, (0, 255, 0), -1)

    # === FPS calculat simplu ===
    text = "YOLO Pose Estimation (ONNX)"
    cv2.putText(frame, text, (10, 30),
                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)

    # === Afișare ===
    cv2.imshow("Live Pose", frame)

    if cv2.waitKey(1) == ord("q"):
        break

cv2.destroyAllWindows()
