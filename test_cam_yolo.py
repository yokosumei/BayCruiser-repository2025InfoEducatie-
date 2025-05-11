from flask import Flask, render_template, Response
from picamera2 import Picamera2
import numpy as np
import cv2
from ultralytics import YOLO

# Inițializăm aplicația Flask
app = Flask(__name__)

# Inițializare model YOLO11s
model = YOLO("my_model.pt")  # Folosește fișierul tău my_model.pt

# Inițializare Picamera2
picam2 = Picamera2()
picam2.configure(picam2.create_video_configuration())  # Configurăm pentru video continuu

# Funcție pentru a captura frame-uri din camera Raspberry Pi
def gen():
    while True:
        frame = picam2.capture_array()  # Capturăm un frame
        results = model(frame)  # Aplicăm detectarea YOLO

        # Obținem coordonatele box-urilor detectate
        for result in results.xywh[0]:
            x, y, w, h, conf, cls = result
            if conf > 0.5:  # Filtrăm box-urile cu o confidență mică
                cv2.rectangle(frame, (int(x - w/2), int(y - h/2)), 
                              (int(x + w/2), int(y + h/2)), (0, 255, 0), 2)
                cv2.putText(frame, f"{model.names[int(cls)]} {conf:.2f}", 
                            (int(x - w/2), int(y - h/2) - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, 
                            (0, 255, 0), 2)

        # Codificăm frame-ul în format JPEG
        ret, jpeg = cv2.imencode('.jpg', frame)
        if not ret:
            continue
        # Trimitem frame-ul codificat în browser
        yield (b'--frame\r\n'
               b'Content-Type: image/jpeg\r\n\r\n' + jpeg.tobytes() + b'\r\n\r\n')

# Ruta pentru pagina principală
@app.route('/')
def index():
    return render_template('index.html')

# Ruta pentru a trimite stream-ul video
@app.route('/video_feed')
def video_feed():
    return Response(gen(), mimetype='multipart/x-mixed-replace; boundary=frame')

if __name__ == '__main__':
    picam2.start()  # Începem capturarea video
    app.run(host='0.0.0.0', port=5000, debug=True)
