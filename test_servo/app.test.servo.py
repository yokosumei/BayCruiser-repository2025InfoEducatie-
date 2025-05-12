from flask import Flask, render_template, Response, jsonify, request
from ultralytics import YOLO
from picamera2 import Picamera2
import cv2
import threading
import time
from gpiozero import AngularServo
from time import sleep

app = Flask(__name__)

# Initialize YOLO model
model = YOLO("my_model.pt")

# Initialize camera
picam2 = Picamera2()
picam2.configure(picam2.create_preview_configuration(main={"format": 'RGB888', "size": (640, 480)}))
picam2.start()

# Initialize servo
servo = AngularServo(18, min_pulse_width=0.0006, max_pulse_width=0.0023)

# Shared state
output_frame = None
lock = threading.Lock()
streaming = False
detection_status = {"detected": False}
popup_triggered = False
alarm_active = False

# Detection thread
def detect_objects():
    global output_frame, streaming, detection_status, popup_triggered, alarm_active
    while streaming:
        frame = picam2.capture_array()

        # Run YOLO detection
        results = model(frame, verbose=False)

        # Draw results
        result_frame = results[0].plot()

        # Check if class 'om_la_inec' was detected
        detected = any(
            model.names[int(cls)] == "om_la_inec" for cls in results[0].boxes.cls
        )

        detection_status["detected"] = detected

        if detected and not popup_triggered and not alarm_active:
            popup_triggered = True

        with lock:
            output_frame = cv2.cvtColor(result_frame, cv2.COLOR_RGB2BGR)

        time.sleep(0.03)  # ~30 FPS
@app.route("/")
def index():
    return render_template("index.test.servo.html")


@app.route("/video_feed")
def video_feed():
    def generate():
        while streaming:
            with lock:
                if output_frame is None:
                    continue
                ret, buffer = cv2.imencode(".jpg", output_frame)
                frame = buffer.tobytes()
            yield (b"--frame\r\n"
                   b"Content-Type: image/jpeg\r\n\r\n" + frame + b"\r\n")
            time.sleep(0.01)
    return Response(generate(), mimetype="multipart/x-mixed-replace; boundary=frame")

@app.route("/start_stream")
def start_stream():
    global streaming
    if not streaming:
        streaming = True
        thread = threading.Thread(target=detect_objects)
        thread.daemon = True
        thread.start()
    return ("", 200)

@app.route("/stop_stream")
def stop_stream():
    global streaming
    streaming = False
    return ("", 200)

@app.route("/detection_status")
def get_detection_status():
    return jsonify(detection_status)

@app.route("/check_popup")
def check_popup():
    global popup_triggered
    if popup_triggered:
        return jsonify({"show": True})
    return jsonify({"show": False})

@app.route("/confirm_action", methods=["POST"])
def confirm_action():
    global popup_triggered, alarm_active
    data = request.get_json()
    if data.get("response") == "yes":
        alarm_active = True
        # Move servo
        servo.angle = 110
        sleep(2)
        servo.angle = 0
        alarm_active = False
    popup_triggered = False
    return jsonify({"status": "ok"})

if __name__ == "__main__":
    app.run(host="0.0.0.0", port=5000)
