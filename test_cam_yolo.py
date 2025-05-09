import cv2
from ultralytics import YOLO
from picamera2 import Picamera2
import time

# Inițializează camera
picam2 = Picamera2()
config = picam2.create_preview_configuration(main={"format": "RGB888", "size": (640, 480)})
picam2.configure(config)
picam2.start()
time.sleep(1)

# Încarcă modelul YOLO
model = YOLO("my_model.pt")

while True:
    frame = picam2.capture_array()

    # Convertire RGBA → RGB dacă e cazul
    if frame.shape[2] == 4:
        frame = cv2.cvtColor(frame, cv2.COLOR_RGBA2RGB)

    results = model.predict(frame, verbose=False)
    annotated = results[0].plot()

    cv2.imshow("YOLO + PiCamera2", annotated)
    if cv2.waitKey(1) & 0xFF == ord("q"):
        break

cv2.destroyAllWindows()
