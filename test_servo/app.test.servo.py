from flask import Flask, render_template
from gpiozero.pins.pigpio import PiGPIOFactory
from gpiozero import AngularServo
from time import sleep

app = Flask(__name__)

factory = PiGPIOFactory()
servo = AngularServo(18, min_angle=0, max_angle=180, min_pulse_width=0.0006, max_pulse_width=0.0023, pin_factory=factory)

@app.route("/")
def home():
    return render_template("index.test.servo.html")

@app.route("/misca")
def misca_servo():
    servo.angle = 110
    sleep(1)
    servo.angle = 0
    return "Colac aruncat|Initiere salvare!"

if __name__ == "__main__":
    app.run(host='0.0.0.0', port=5000, debug=True)

