import cv2
import numpy as np
import ncnn
from picamera2 import Picamera2

# Config
MODEL_PARAM = "yolo11n-pose-opt.param"
MODEL_BIN = "yolo11n-pose-opt.bin"
IMG_SIZE = 640
CONF_THRESH = 0.5

# Initialize NCNN
net = ncnn.Net()
net.opt.use_vulkan_compute = False  # GPU off for Pi
net.load_param(MODEL_PARAM)
net.load_model(MODEL_BIN)

# Start PiCamera2
picam2 = Picamera2()
picam2.preview_configuration.main.size = (IMG_SIZE, IMG_SIZE)
picam2.preview_configuration.main.format = "RGB888"
picam2.preview_configuration.align()
picam2.configure("preview")
picam2.start()

# Pose drawing (17 keypoints COCO format)
def draw_pose(img, kpts, conf_thresh=CONF_THRESH):
    for i in range(0, len(kpts), 3):
        x = int(kpts[i] * img.shape[1])
        y = int(kpts[i + 1] * img.shape[0])
        conf = kpts[i + 2]
        if conf > conf_thresh:
            cv2.circle(img, (x, y), 3, (0, 255, 0), -1)

while True:
    frame = picam2.capture_array()
    resized = cv2.resize(frame, (IMG_SIZE, IMG_SIZE))

    # Convert to NCNN format
    mat = ncnn.Mat.from_pixels(resized, ncnn.Mat.PixelType.PIXEL_RGB, IMG_SIZE, IMG_SIZE)

    # Run inference
    ex = net.create_extractor()
    ex.input("images", mat)

    ret, out = ex.extract("output0")  # Confirmed name

    if ret == 0 and out.w == 51:  # 17 keypoints × 3 (x, y, conf)
        keypoints = np.array(out)[0]
        draw_pose(resized, keypoints)

    # Show frame
    cv2.imshow("YOLOv11 Pose NCNN", resized)
    if cv2.waitKey(1) & 0xFF == ord("q"):
        break

picam2.stop()
cv2.destroyAllWindows()
