from flask import Flask
from gpiozero import AngularServo
from time import sleep

app = Flask(__name__)

servo1 = AngularServo(18, min_pulse_width=0.0006, max_pulse_width=0.0023)
servo2 = AngularServo(17, min_pulse_width=0.0006, max_pulse_width=0.0023)

@app.route("/")
def index():
    return render_template("index.test.servo.html")

@app.route("misca")
def activate():
    servo1.angle = 110
    servo2.angle = 110
    sleep(2)
    servo1.angle = 90
    servo2.angle = 90

if __name__ == "__main__":
    app.run(host="0.0.0.0", port=5000)
