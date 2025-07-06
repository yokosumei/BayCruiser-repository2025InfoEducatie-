import cv2
import numpy as np
import ncnn

# === Configurație model ===
MODEL_PARAM = "yolo11n-pose-opt.param"
MODEL_BIN = "yolo11n-pose-opt.bin"
IMG_SIZE = 640
CONF_THRESHOLD = 0.5

# === Inițializare NCNN ===
net = ncnn.Net()
net.load_param(MODEL_PARAM)
net.load_model(MODEL_BIN)

# === Inițializare cameră ===
cap = cv2.VideoCapture(0)  # index 0 = camera default
cap.set(cv2.CAP_PROP_FRAME_WIDTH, IMG_SIZE)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, IMG_SIZE)

# === Funcție pentru desenare keypoints (COCO format - 17 puncte) ===
def draw_pose(frame, keypoints, threshold=CONF_THRESHOLD):
    for i in range(0, len(keypoints), 3):
        x = int(keypoints[i] * frame.shape[1])
        y = int(keypoints[i+1] * frame.shape[0])
        conf = keypoints[i+2]
        if conf > threshold:
            cv2.circle(frame, (x, y), 4, (0, 255, 0), -1)

# === Loop video ===
while True:
    ret, frame = cap.read()
    if not ret:
        break

    # Redimensionare + NCNN input
    img = cv2.resize(frame, (IMG_SIZE, IMG_SIZE))
    mat = ncnn.Mat.from_pixels_resize(img, ncnn.Mat.PixelType.PIXEL_BGR, IMG_SIZE, IMG_SIZE)

    # Inferență
    ex = net.create_extractor()
    ex.input("images", mat)
    
    ret, out = ex.extract("output")  # poate fi și "output1", "cls", depinde de model
    
    # Conversie NCNN → NumPy (dacă e tip [1, 17*3])
    if out.w == 51:  # 17 keypoints x (x,y,conf)
        keypoints = np.array(out)[0]  # [1, 51]
        draw_pose(img, keypoints)

    # Afișare
    cv2.imshow("YOLOv11 Pose - Live", img)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
