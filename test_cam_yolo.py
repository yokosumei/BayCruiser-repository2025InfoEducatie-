from flask import Flask, render_template, Response, jsonify
from ultralytics import YOLO
from picamera2 import Picamera2
import RPi.GPIO as GPIO
import numpy as np
import threading
import cv2
import time
import atexit
# from dronekit import connect, VehicleMode, LocationGlobalRelative

"""
# === DroneKit setup ===
connection_string = '/dev/ttyUSB0'
baud_rate = 57600
print("Connecting to vehicle...")
vehicle = connect(connection_string, baud=baud_rate, wait_ready=False)

def arm_and_takeoff(target_altitude):
    print("Checking pre-arm conditions...")
    while not vehicle.is_armable:
        print(" Waiting for vehicle to initialise...")
        time.sleep(1)
    print("Arming motors...")
    vehicle.mode = VehicleMode("GUIDED")
    vehicle.armed = True
    while not vehicle.armed:
        print(" Waiting for arming...")
        time.sleep(1)
    print("Taking off!")
    vehicle.simple_takeoff(target_altitude)
    while True:
        alt = vehicle.location.global_relative_frame.alt
        print(" Altitude: ", alt)
        if alt >= target_altitude * 0.95:
            print("Reached target altitude")
            break
        time.sleep(1)

def land_drone():
    vehicle.mode = VehicleMode("LAND")
    while vehicle.armed:
        time.sleep(1)
    print("Landed and disarmed.")
    vehicle.close()
"""

# === Flask App Setup ===
app = Flask(__name__)
model = YOLO("my_model.pt")
picam2 = Picamera2()
picam2.configure(picam2.create_video_configuration(main={"format": "RGB888", "size": (640, 480)}))
picam2.start()

# GPIO setup for servos
GPIO.setmode(GPIO.BOARD)
GPIO.setup(11, GPIO.OUT)
GPIO.setup(12, GPIO.OUT)
servo1 = GPIO.PWM(11, 50)
servo2 = GPIO.PWM(12, 50)

# Inițializare fără jitter: centru (7.5), apoi oprim semnalul
servo1.start(7.5)
servo2.start(7.5)
time.sleep(0.3)
servo1.ChangeDutyCycle(0)
servo2.ChangeDutyCycle(0)

streaming = False
lock = threading.Lock()
output_frame = None
detected_flag = False
popup_sent = False
last_detection_time = 0

def cleanup():
    servo1.stop()
    servo2.stop()
    GPIO.cleanup()

atexit.register(cleanup)

# Activează servo-urile spre 90° dreapta
def activate_servos():
    servo1.ChangeDutyCycle(12.5)
    servo2.ChangeDutyCycle(12.5)
    time.sleep(3)
    servo1.ChangeDutyCycle(7.5)
    servo2.ChangeDutyCycle(7.5)
    time.sleep(3)
    servo1.ChangeDutyCycle(0)
    servo2.ChangeDutyCycle(0)

def blank_frame():
    img = np.zeros((480, 640, 3), dtype=np.uint8)
    _, buffer = cv2.imencode('.jpg', img)
    return buffer.tobytes()

def detect_objects():
    global output_frame, streaming, detected_flag, popup_sent, last_detection_time

    while streaming:
        frame = picam2.capture_array()
        results = model(frame, verbose=False)
        annotated = results[0].plot()

        names = results[0].names
        class_ids = results[0].boxes.cls.tolist()

        for i, cls_id in enumerate(class_ids):
            if names[int(cls_id)] == "om_la_inec":
                detected_flag = True
                popup_sent = True
                last_detection_time = time.time()
                activate_servos()

                # Dacă ai `calculate_offset` și `move_towards` definite, le poți reactiva
                # box = results[0].boxes[i]
                # dx_cm, dy_cm = calculate_offset(box)
                # move_towards(dx_cm, dy_cm)

        if time.time() - last_detection_time > 5:
            detected_flag = False
            popup_sent = False

        with lock:
            output_frame = cv2.imencode('.jpg', annotated)[1].tobytes()

        time.sleep(0.05)

@app.route("/")
def index():
    return render_template("index.html")

@app.route("/video_feed")
def video_feed():
    def generate():
        global output_frame
        while True:
            if not streaming:
                time.sleep(0.1)
                continue
            with lock:
                frame = output_frame if output_frame is not None else blank_frame()
            yield (b"--frame\r\n"
                   b"Content-Type: image/jpeg\r\n\r\n" + frame + b"\r\n")
            time.sleep(0.05)
    return Response(generate(), mimetype="multipart/x-mixed-replace; boundary=frame")

@app.route("/start_stream")
def start_stream():
    global streaming
    if not streaming:
        streaming = True
        thread = threading.Thread(target=detect_objects)
        thread.daemon = True
        thread.start()
    return jsonify({"status": "started"})

@app.route("/stop_stream")
def stop_stream():
    global streaming
    streaming = False
    return jsonify({"status": "stopped"})

@app.route("/detection_status")
def detection_status():
    return jsonify({"detected": popup_sent})

@app.route("/misca")
def activate():
    activate_servos()
    return "Servomotor activat"

@app.route("/takeoff")
def takeoff():
    return "Drone Takeoff (dezactivat temporar)"

@app.route("/land")
def land():
    return "Drone Landing (dezactivat temporar)"

if __name__ == "__main__":
    app.run(host="0.0.0.0", port=5000)
