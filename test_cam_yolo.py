import cv2
import numpy as np
import tflite_runtime.interpreter as tflite
from picamera2 import Picamera2
from time import sleep
from gpiozero import AngularServo

# Config
MODEL_PATH = "weights/my_model_final_int8.tflite"
CLASSES = ["om_la_inec", "inotator"]
CONFIDENCE_THRESHOLD = 0.5

# Servo
servo = AngularServo(18, min_pulse_width=0.0006, max_pulse_width=0.0023)
servo_activated = False

# TFLite
interpreter = tflite.Interpreter(model_path=MODEL_PATH)
interpreter.allocate_tensors()
input_details = interpreter.get_input_details()
output_details = interpreter.get_output_details()
input_shape = input_details[0]['shape']
input_dtype = input_details[0]['dtype']
_, input_height, input_width, _ = input_shape

# Cameră
picam2 = Picamera2()
picam2.configure(picam2.create_video_configuration(main={"size": (640, 480)}))
picam2.start()
sleep(2)

# Funcții
def preprocess(frame):
    image = cv2.resize(frame, (input_width, input_height))
    if input_dtype == np.float32:
        return np.expand_dims(image, axis=0).astype(np.float32) / 255.0
    elif input_dtype == np.uint8:
        return np.expand_dims(image, axis=0).astype(np.uint8)

def postprocess(output_data, original_shape):
    output_data = output_data.reshape((6, 8400)).T  # (8400, 6)
    h, w, _ = original_shape
    predictions = []
    for x, y, w_box, h_box, conf, cls_id in output_data:
        if conf > CONFIDENCE_THRESHOLD:
            x1 = int((x - w_box/2) * w)
            y1 = int((y - h_box/2) * h)
            x2 = int((x + w_box/2) * w)
            y2 = int((y + h_box/2) * h)
            predictions.append((x1, y1, x2, y2, float(conf), int(cls_id)))
    return predictions

def draw_detections(frame, detections):
    global servo_activated
    for x1, y1, x2, y2, conf, cls_id in detections:
        label = f"{CLASSES[cls_id]} {conf:.2f}"
        cv2.rectangle(frame, (x1, y1), (x2, y2), (0,255,0), 2)
        cv2.putText(frame, label, (x1, y1-10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,255,0), 2)

        if CLASSES[cls_id] == "om_la_inec" and not servo_activated:
            print("[ACTIVARE SERVO]")
            servo.angle = 110
            sleep(1)
            servo.angle = 90
            sleep(1)
            servo_activated = True

# Loop principal
print("[INFO] Apasă Q pentru a ieși.")
try:
    while True:
        frame = picam2.capture_array()
        input_tensor = preprocess(frame)
        interpreter.set_tensor(input_details[0]['index'], input_tensor)
        interpreter.invoke()
        output_data = interpreter.get_tensor(output_details[0]['index'])[0]
        detections = postprocess(output_data, frame.shape)
        draw_detections(frame, detections)

        cv2.imshow("TFLite YOLOv11s", frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
finally:
    picam2.stop()
    cv2.destroyAllWindows()
