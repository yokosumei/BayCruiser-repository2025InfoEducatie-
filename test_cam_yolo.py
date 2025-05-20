from flask import Flask, render_template, Response, jsonify
from ultralytics import YOLO
from picamera2 import Picamera2
import cv2
import threading
import time
import RPi.GPIO as GPIO

app = Flask(__name__)

# YOLOv8 modelul tău (schimbă dacă ai altul)
model = YOLO("yolo11s.pt")

# Camera & servo setup
picam2 = Picamera2()
picam2.configure(picam2.create_preview_configuration(main={"format": 'XRGB8888', "size": (640, 480)}))

# GPIO pentru servo
SERVO_PIN = 17
GPIO.setmode(GPIO.BCM)
GPIO.setup(SERVO_PIN, GPIO.OUT)
servo = GPIO.PWM(SERVO_PIN, 50)
servo.start(0)

# Variabile globale
streaming = False
output_frame = None
lock = threading.Lock()
detection_flag = False


def move_servo(angle):
    duty = angle / 18 + 2.5
    GPIO.output(SERVO_PIN, True)
    servo.ChangeDutyCycle(duty)
    time.sleep(0.5)
    GPIO.output(SERVO_PIN, False)
    servo.ChangeDutyCycle(0)


def detect():
    global output_frame, detection_flag, streaming

    picam2.start()

    while streaming:
        frame = picam2.capture_array()
        results = model.predict(source=frame, show=False, stream=False, conf=0.5)

        if results:
            result = results[0]
            annotated_frame = result.plot()

            # Verifică dacă obiectul "om_la_inec" este detectat
            detection_flag = False
            for cls in result.names:
                if result.names[cls] == "om_la_inec":
                    detection_flag = True
                    break
        else:
            annotated_frame = frame
            detection_flag = False

        with lock:
            output_frame = cv2.cvtColor(annotated_frame, cv2.COLOR_RGB2BGR)

    picam2.stop()


def generate_frames():
    global output_frame
    while True:
        with lock:
            if output_frame is None:
                continue
            ret, buffer = cv2.imencode('.jpg', output_frame)
            frame = buffer.tobytes()
        yield (b'--frame\r\n'
               b'Content-Type: image/jpeg\r\n\r\n' + frame + b'\r\n')


@app.route('/')
def index():
    return render_template('index.html')


@app.route('/start_stream')
def start_stream():
    global streaming
    if not streaming:
        streaming = True
        t = threading.Thread(target=detect)
        t.daemon = True
        t.start()
    return ('', 204)


@app.route('/stop_stream')
def stop_stream():
    global streaming
    streaming = False
    return ('', 204)


@app.route('/video_feed')
def video_feed():
    return Response(generate_frames(),
                    mimetype='multipart/x-mixed-replace; boundary=frame')


@app.route('/detection_status')
def detection_status():
    return jsonify({"detected": detection_flag})


@app.route('/misca')
def misca():
    move_servo(110)
    return "Servomotorul s-a mișcat la 110°!"


@app.route('/shutdown', methods=['POST'])
def shutdown():
    global streaming
    streaming = False
    servo.stop()
    GPIO.cleanup()
    func = request.environ.get('werkzeug.server.shutdown')
    if func:
        func()
    return "Server oprit."


if __name__ == '__main__':
    app.run(host='0.0.0.0', port=5000, debug=True)
