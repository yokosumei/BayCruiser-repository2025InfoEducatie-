from flask import Flask, render_template, Response
from picamera2 import Picamera2
import cv2
import threading
import time

app = Flask(__name__)

picam2 = Picamera2()
picam2.configure(picam2.create_preview_configuration(main={"format": 'RGB888', "size": (640, 480)}))
picam2.start()

output_frame = None
lock = threading.Lock()
streaming = False

def generate_fake_stream():
    global output_frame, streaming
    while streaming:
        frame = picam2.capture_array()

        # Desenează un pătrat roșu
        cv2.rectangle(frame, (100, 100), (200, 200), (255, 0, 0), 3)

        with lock:
            output_frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)

        time.sleep(0.03)

@app.route("/")
def index():
    return render_template("index.html")

@app.route("/video_feed")
def video_feed():
    def generate():
        while streaming:
            with lock:
                if output_frame is None:
                    continue
                ret, buffer = cv2.imencode(".jpg", output_frame)
                if not ret:
                    continue
                frame = buffer.tobytes()
            yield (b"--frame\r\n"
                   b"Content-Type: image/jpeg\r\n\r\n" + frame + b"\r\n")
            time.sleep(0.03)
    return Response(generate(), mimetype="multipart/x-mixed-replace; boundary=frame")

@app.route("/start_stream")
def start_stream():
    global streaming
    if not streaming:
        streaming = True
        print("Stream de test pornit")
        thread = threading.Thread(target=generate_fake_stream)
        thread.daemon = True
        thread.start()
    return ("", 200)

@app.route("/stop_stream")
def stop_stream():
    global streaming
    streaming = False
    print("Stream oprit")
    return ("", 200)

if __name__ == "__main__":
    app.run(host="0.0.0.0", port=5000)
