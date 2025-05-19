import cv2
import numpy as np
import tensorflow as tf

# Încarcă modelul TFLite
interpreter = tf.lite.Interpreter(model_path="my_model_final_int8 - Copy.tflite")
interpreter.allocate_tensors()

input_details = interpreter.get_input_details()
output_details = interpreter.get_output_details()

# Informații despre intrare
height = input_details[0]['shape'][1]
width = input_details[0]['shape'][2]

# Funcție de preprocesare
def preprocess(frame):
    resized = cv2.resize(frame, (width, height))
    input_data = np.expand_dims(resized, axis=0).astype(np.uint8)  # dacă modelul e quantizat INT8
    return input_data

# Camera
cap = cv2.VideoCapture(0)  # 0 = prima cameră (poate fi schimbat cu /dev/video1 etc.)

while True:
    ret, frame = cap.read()
    if not ret:
        break

    input_data = preprocess(frame)
    interpreter.set_tensor(input_details[0]['index'], input_data)
    interpreter.invoke()

    # Extrage outputurile (de obicei: boxe, scoruri, clase, număr de detecții)
    boxes = interpreter.get_tensor(output_details[0]['index'])[0]       # shape: (N, 4)
    classes = interpreter.get_tensor(output_details[1]['index'])[0]     # shape: (N,)
    scores = interpreter.get_tensor(output_details[2]['index'])[0]      # shape: (N,)
    count = int(interpreter.get_tensor(output_details[3]['index'])[0])  # N detecții

    # Desenează detecțiile
    for i in range(count):
        if scores[i] > 0.5:  # threshold
            ymin, xmin, ymax, xmax = boxes[i]
            (left, top, right, bottom) = (
                int(xmin * frame.shape[1]),
                int(ymin * frame.shape[0]),
                int(xmax * frame.shape[1]),
                int(ymax * frame.shape[0])
            )
            cv2.rectangle(frame, (left, top), (right, bottom), (0, 255, 0), 2)
            label = f"Clasa: {int(classes[i])} - {scores[i]:.2f}"
            cv2.putText(frame, label, (left, top - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)

    cv2.imshow("Detecție TFLite", frame)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
