import cv2
import numpy as np
import ncnn
from picamera2 import Picamera2

# === Configurație model NCNN ===
MODEL_PARAM = "yolo11n-pose-opt.param"
MODEL_BIN = "yolo11n-pose-opt.bin"
IMG_SIZE = 640
CONF_THRESH = 0.5

# === Inițializare NCNN ===
net = ncnn.Net()
net.opt.use_vulkan_compute = False  # GPU off (safe for Pi)
net.load_param(MODEL_PARAM)
net.load_model(MODEL_BIN)

# === Inițializare cameră Pi ===
picam2 = Picamera2()
camera_config = picam2.create_preview_configuration(main={"format": "RGB888", "size": (IMG_SIZE, IMG_SIZE)})
picam2.configure(camera_config)
picam2.start()

# === Funcție desen keypoints (17 x,y,conf) ===
def draw_pose(img, kpts, conf_thresh=CONF_THRESH):
    for i in range(0, len(kpts), 3):
        x = int(kpts[i] * img.shape[1])
        y = int(kpts[i + 1] * img.shape[0])
        conf = kpts[i + 2]
        if conf > conf_thresh:
            cv2.circle(img, (x, y), 3, (0, 255, 0), -1)

# === Loop live inferență ===
while True:
    frame = picam2.capture_array()
    
    # Verificare dimensiune și tip
    if frame.shape[:2] != (IMG_SIZE, IMG_SIZE):
        frame = cv2.resize(frame, (IMG_SIZE, IMG_SIZE))

    # Convert to NCNN input format
    mat = ncnn.Mat.from_pixels(frame, ncnn.Mat.PixelType.PIXEL_RGB, IMG_SIZE, IMG_SIZE)

    # Inferență
    ex = net.create_extractor()
    ex.input("images", mat)

    ret, out = ex.extract("output0")  # Confirmat din fișierul tău .param

    if ret == 0 and out.w == 51:  # 17 keypoints × (x, y, conf)
        keypoints = np.array(out)[0]
        draw_pose(frame, keypoints)

    # Afișare live
    cv2.imshow("YOLOv11 Pose - NCNN + PiCamera2", frame)
    if cv2.waitKey(1) & 0xFF == ord("q"):
        break

picam2.stop()
cv2.destroyAllWindows()
