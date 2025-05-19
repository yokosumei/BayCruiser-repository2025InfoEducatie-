import cv2
import numpy as np
import tensorflow as tf

# === CONFIG ===
MODEL_PATH = "my_model_final_int8.tflite"
LABEL = "om_la_inec"  # schimbi dacă e alt label
THRESHOLD = 0.4

# === Load TFLite Model ===
interpreter = tf.lite.Interpreter(model_path=MODEL_PATH)
interpreter.allocate_tensors()
input_details = interpreter.get_input_details()
output_details = interpreter.get_output_details()

input_shape = input_details[0]['shape'][1:3]

# === Video Capture ===
cap = cv2.VideoCapture(0)  # 0 pentru camera implicită (USB/Pi cam)

while True:
    ret, frame = cap.read()
    if not ret:
        break

    input_img = cv2.resize(frame, tuple(input_shape))
    input_img = np.expand_dims(input_img, axis=0).astype(np.uint8)

    interpreter.set_tensor(input_details[0]['index'], input_img)
    interpreter.invoke()

    # Preluare rezultate brute (adaptat în funcție de model)
    boxes = interpreter.get_tensor(output_details[0]['index'])[0]  # (N, 4)
    classes = interpreter.get_tensor(output_details[1]['index'])[0]  # (N,)
    scores = interpreter.get_tensor(output_details[2]['index'])[0]  # (N,)

    for i in range(len(scores)):
        if scores[i] > THRESHOLD:
            ymin, xmin, ymax, xmax = boxes[i]
            (h, w) = frame.shape[:2]
            (left, top, right, bottom) = (int(xmin * w), int(ymin * h), int(xmax * w), int(ymax * h))
            cv2.rectangle(frame, (left, top), (right, bottom), (0, 255, 0), 2)
            cv2.putText(frame, f"{LABEL} ({scores[i]:.2f})", (left, top - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

    cv2.imshow("TFLite Detection", frame)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
