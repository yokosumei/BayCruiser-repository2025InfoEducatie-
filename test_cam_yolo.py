import cv2
import numpy as np
import tflite_runtime.interpreter as tflite
from picamera2 import Picamera2
import time

# Încarcă modelul TFLite
interpreter = tflite.Interpreter(model_path="weights/my_model_final.tflite")
interpreter.allocate_tensors()

input_details = interpreter.get_input_details()
output_details = interpreter.get_output_details()
_, input_height, input_width, _ = input_details[0]['shape']

# Etichete personalizate
CLASSES = ["om_la_inec", "inotator"]
CONFIDENCE_THRESHOLD = 0.5

# Initializează camera
picam2 = Picamera2()
picam2.configure(picam2.create_video_configuration(main={"size": (640, 480)}))
picam2.start()

while True:
    frame = picam2.capture_array()
    input_image = cv2.resize(frame, (input_width, input_height))
    input_tensor = np.expand_dims(input_image, axis=0).astype(np.uint8)

    interpreter.set_tensor(input_details[0]['index'], input_tensor)
    interpreter.invoke()

    boxes = interpreter.get_tensor(output_details[0]['index'])[0]
    classes = interpreter.get_tensor(output_details[1]['index'])[0]
    scores = interpreter.get_tensor(output_details[2]['index'])[0]

    for i in range(len(scores)):
        if scores[i] > CONFIDENCE_THRESHOLD:
            ymin, xmin, ymax, xmax = boxes[i]
            h, w, _ = frame.shape
            (left, top, right, bottom) = (
                int(xmin * w), int(ymin * h),
                int(xmax * w), int(ymax * h)
            )
            label = f"{CLASSES[int(classes[i])]}: {scores[i]:.2f}"
            cv2.rectangle(frame, (left, top), (right, bottom), (0, 255, 0), 2)
            cv2.putText(frame, label, (left, top - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

    cv2.imshow("Test Camera YOLOv11s", frame)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cv2.destroyAllWindows()
picam2.stop()
