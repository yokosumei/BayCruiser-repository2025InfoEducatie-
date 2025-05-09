from picamera2 import Picamera2
import cv2
from ultralytics import YOLO

# Inițializează camera
picam2 = Picamera2()
picam2.configure(picam2.preview_configuration(main={"size": (640, 480)}))
picam2.start()

# Încarcă modelul
model = YOLO("my_model.pt")  # sau "yolov5s.pt", etc.

while True:
    # Capturează un frame de la cameră
    frame = picam2.capture_array()

    # Rulează detecția
    results = model.predict(source=frame, show=True, conf=0.4)

    # Așteaptă tasta 'q' ca să oprească
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Eliberează resursele
cv2.destroyAllWindows()
picam2.close()
