from picamera2 import Picamera2
from ultralytics import YOLO
import cv2

picam2 = Picamera2()
picam2.configure(picam2.create_preview_configuration(main={"size": (640, 480)}))
picam2.start()

model = YOLO("my_model.pt")  # sau numele tău

while True:
    frame = picam2.capture_array()
    frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)  # YOLO așteaptă RGB
    results = model(frame_rgb)  # asta ar trebui să meargă acum

    annotated = results[0].plot()
    cv2.imshow("YOLO Preview", annotated)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

picam2.close()
cv2.destroyAllWindows()
