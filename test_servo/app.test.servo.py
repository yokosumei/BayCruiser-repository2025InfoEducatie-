from flask import Flask, render_template
from gpiozero import AngularServo
from time import sleep

app = Flask(__name__, template_folder="templates", static_folder="static")

servo = AngularServo(18, min_pulse_width=0.0006, max_pulse_width=0.0023)

@app.route("/")
def home():
    return render_template("index.test.servo.html")

@app.route("/misca")
def misca_servo():
    servo.angle = 110
    sleep(1)
    servo.angle = 0
    return "Servo mișcat!"
