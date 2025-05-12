from gpiozero.pins.pigpio import PiGPIOFactory
from gpiozero import AngularServo

factory = PiGPIOFactory()
servo = AngularServo(18, min_pulse_width=0.0006, max_pulse_width=0.0023, pin_factory=factory)


app = Flask(__name__, template_folder="templates", static_folder="static")


@app.route("/")
def home():
    return render_template("index.test.servo.html")

@app.route("/misca")
def misca_servo():
    servo.angle = 110
    sleep(1)
    servo.angle = 0
    return "Servo mișcat!"
