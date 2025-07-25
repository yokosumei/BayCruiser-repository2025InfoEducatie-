from matplotlib.pyplot import step
from flask import Flask, render_template, Response, request, jsonify
from flask_socketio import SocketIO
from werkzeug.utils import secure_filename
import onnxruntime as ort
import uuid
import joblib
import eventlet
from ultralytics import YOLO
from picamera2 import Picamera2
import RPi.GPIO as GPIO
import numpy as np
import threading
import cv2
import time
import atexit
import os
import logging

from collections import deque
from dronekit import connect, VehicleMode, LocationGlobalRelative
import math
from queue import Queue
from pymavlink import mavutil
from threading import Event





logging.basicConfig(level=logging.INFO, format='[%(levelname)s] (%(threadName)s) %(message)s')


app = Flask(__name__)
socketio = SocketIO(app, async_mode='threading')

app.config['UPLOAD_FOLDER'] = 'static/uploads'
os.makedirs(app.config['UPLOAD_FOLDER'], exist_ok=True)


picam2 = Picamera2()
picam2.configure(picam2.create_video_configuration(main={"format": "RGB888", "size": (640, 480)}))
picam2.start()

GPIO.setmode(GPIO.BOARD)
GPIO.setup(11, GPIO.OUT)
GPIO.setup(12, GPIO.OUT)
GPIO.setup(29, GPIO.OUT)
GPIO.setup(31, GPIO.OUT)
servo1 = GPIO.PWM(11, 50)
servo2 = GPIO.PWM(12, 50)
servo1.start(7.5)
servo2.start(7.5)
time.sleep(0.3)
servo1.ChangeDutyCycle(0)
servo2.ChangeDutyCycle(0)

right_stream_type = "daria"


detection_thread = None
stop_detection_event = Event()

detection_liv_thread = None
stop_detection_liv_event = Event()

segmnetation_thread= None
stop_segmentation_event = Event()

pose_thread= None
stop_pose_event = Event()


stop_takeoff_event = Event()

smart_stream_mode = False



pose_triggered = False
pose_thread_started = False
streaming = False
frame_lock = threading.Lock()
output_lock = threading.Lock()
mar_lock = threading.Lock()
seg_lock = threading.Lock()
pose_lock = threading.Lock()

xgb_model = joblib.load("models/xgb_pipeline.joblib")
label_map = {0: "inot", 1: "inec"}

# Clasifică activitatea ca înot sau posibil înec, pe baza unui vector de 1020(flatten din 30 frame-uri * 34 keypoints) coordonate extrase din keypoints.
def xgb_predict(vector_1020):
    vector_1020 = np.array(vector_1020).reshape(1, -1)
    prediction = xgb_model.predict(vector_1020)[0]
    return label_map.get(prediction, str(prediction))

output_frame_pose = None
output_lock_pose = threading.Lock()
pose_sequence_buffer = deque(maxlen=30)
frame_buffer = None
output_frame = None
yolo_output_frame = None
detected_flag = False
popup_sent = False
last_detection_time = 0
detection_frame_skip = 2
frame_counter = 0
event_location = None  # Fixed: Initialize event_location
mar_output_frame = None
seg_output_frame = None
pose_output_frame = None
pose_triggered = False


def cleanup():
    try: servo1.stop()
    except: pass
    try: servo2.stop()
    except: pass
    try: picam2.stop()
    except: pass
    try: GPIO.cleanup()
    except: pass

atexit.register(cleanup)
# Pornește un thread daemon care rulează funcția specificată.
# Este folosit pentru a executa procese paralele (ex: camere, inferență, streaming) fără a bloca aplicația.
def start_thread(func, name="WorkerThread"):
    t = threading.Thread(target=func, name=name, daemon=True)
    t.start()
    return t
# Activează și apoi resetează două servomotoare pentru a executa aruncarea colacului.
def activate_servos():
    print("Activare servomotoare")
    GPIO.output(29, GPIO.HIGH) 
    GPIO.output(31, GPIO.HIGH)  

    servo1.ChangeDutyCycle(12.5)
    servo2.ChangeDutyCycle(2.5)
    time.sleep(0.3)
    servo1.ChangeDutyCycle(0)
    servo2.ChangeDutyCycle(0)
    time.sleep(2)
    servo1.ChangeDutyCycle(7.5)
    servo2.ChangeDutyCycle(7.5)
    time.sleep(0.3)
    servo1.ChangeDutyCycle(0)
    servo2.ChangeDutyCycle(0)
    GPIO.output(29, GPIO.LOW)
    GPIO.output(31, GPIO.LOW)
    print("Finalizare activare servomotoare")

# Generează un cadru negru (blank) de 640x480.
# Folosit când nu există frame disponibil pentru a evita erorile în stream.
def blank_frame():
    img = np.zeros((480, 640, 3), dtype=np.uint8)
    _, buffer = cv2.imencode('.jpg', img)
    return buffer.tobytes()

# === GPS SIMULATOR ===
USE_SIMULATOR = False

class GPSValue:
    def __init__(self, lat, lon, alt):
        self.lat = lat
        self.lon = lon
        self.alt = alt
        
class BaseGPSProvider:
    def get_location(self):
        raise NotImplementedError()
    def close(self):
        pass        

class MockGPSProvider:
    def __init__(self):
        self.coordinates = deque([
            (44.4391 + i * 0.0001, 26.0961 + i * 0.0001, 80.0) for i in range(20)
        ])

    def get_location(self):
        try:
            lat, lon, alt = self.coordinates[0]
            self.coordinates.rotate(-1)
            logging.debug(f"[MOCK GPS] Coordonată returnată: lat={lat}, lon={lon}, alt={alt}")
            return GPSValue(lat, lon, alt)
        except Exception as e:
            logging.exception("[MOCK GPS] Eroare la generarea coordonatei")
            return GPSValue(None, None, None)

# === DroneKit setup ===


class DroneKitGPSProvider(BaseGPSProvider):
    def __init__(self, connection_string='/dev/ttyUSB0', baud_rate=57600, bypass=False):
        self.bypass = bypass
        self.vehicle = None
        self.location = GPSValue(None, None, None)
        self.connected = False
        self.connection_string = connection_string
        self.baud_rate = baud_rate

        self.command_queue = Queue()
        self.current_state = "IDLE"
        self.current_command = None
        self.state_machine_thread = start_thread(self._run_state_machine, "DroneStateMachine")

    def _connect_drone(self):
        try:
            print("[DroneKitGPSProvider] Conectare la dronă...")
            self.vehicle = connect(self.connection_string, baud=self.baud_rate, wait_ready=False)
            time.sleep(1)
            self.vehicle.add_attribute_listener('location.global_frame', self.gps_callback)
            time.sleep(1)
            self.connected = True
            logging.info("[DroneKitGPSProvider] Conectare la Pixhawk completă")
        except Exception as e:
            logging.error(f"[DroneKitGPSProvider] Eroare la conectare: {e}")
            self.connected = False

    def gps_callback(self, self_ref, attr_name, value):
        try:
            self.location = GPSValue(value.lat, value.lon, value.alt)
            logging.debug(f"[GPS] lat={value.lat}, lon={value.lon}, alt={value.alt}")
        except Exception as e:
            logging.exception("[GPS] Eroare în gps_callback")
    def get_location(self):
        return self.location
    def ensure_connection(self):
        if not self.connected or self.vehicle is None:
            raise Exception("Drone not connected")
        return True

    def arm_and_takeoff(self, target_altitude, vehicle_mode):
        global stop_takeoff_event
        try:
            self.ensure_connection()
        except:
            return "[DroneKit] Drone not connected"
        stop_takeoff_event.clear()
        print("[DroneKit] Armare..........in mod ", vehicle_mode)
        self.vehicle.mode = VehicleMode(vehicle_mode)

        if not self.wait_until_ready():
            return "[DroneKit] Nu e armabilă. Ieșire."

        print(f"[DroneKit] Decolare la {target_altitude}m...")
        self.vehicle.simple_takeoff(target_altitude)

        while not stop_takeoff_event.is_set():
            alt = self.vehicle.location.global_relative_frame.alt
            print("Altitudine (față de nivelul mării):", self.vehicle.location.global_frame.alt)
            print("Altitudine relativă (față de decolare):", alt)
            print("Altitudine target_altitude:", target_altitude * 0.95)
            if alt >= target_altitude * 0.95:
                print("[DroneKit] Altitudine atinsă.")
                break
            time.sleep(1)

        return "Drone Takeoff"

    def wait_until_ready(self, timeout=30):

        if not self.ensure_connection():
            return False
        # print("[INFO] Trimit comanda de armare forțată (MAV_CMD_COMPONENT_ARM_DISARM)...")
        # self.vehicle._master.mav.command_long_send(
        #     self.vehicle._master.target_system,
        #     self.vehicle._master.target_component,
        #     mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        #     0,          # confirmation
        #     1,          # param1: 1=arm, 0=disarm
        #     21196,      # param2: magic code pentru override
        #     0, 0, 0, 0, 0
        # )
        self.vehicle.armed = True

        print("..............[DroneKit] Așteptăm ca drona să fie armabilă...")
        start = time.time()
        while not self.vehicle.armed:

            print("..  -> Mode:", self.vehicle.mode.name)
            print("..  -> Is armable:", self.vehicle.is_armable)
            print("..  -> ARMED:", self.vehicle.armed)
            print("..  -> EKF OK:", self.vehicle.ekf_ok)
            print("..  -> GPS fix:", self.vehicle.gps_0.fix_type)
            print("..  -> Sateliți:", self.vehicle.gps_0.satellites_visible)
            print("..  -> Sistem:", self.vehicle.system_status.state)
            print("Altitudine (față de nivelul mării):", self.vehicle.location.global_frame.alt)
            print("Altitudine relativă (față de decolare):", self.vehicle.location.global_relative_frame.alt)

            if time.time() - start > timeout:
                print(".....[DroneKit] Timeout atins. Nu e armabilă.")
                return False
            time.sleep(1)

        print("[DroneKit] Drona este gata.")
        return True

    def land_drone(self):
        global stop_takeoff_event

        try:
            self.ensure_connection()
        except:
            return "[DroneKit] Drone not connected"

        print("[DroneKit] Aterizare..bbbbbbbbb.")
        stop_takeoff_event.set()
        print("[DroneKit] Aterizare...")
        try:
            self.vehicle.flush()
        except Exception as e:
            print(f"[WARN] flush() failed: {e}")
        

        while self.vehicle.armed:
            alt = self.vehicle.location.global_relative_frame.alt
            print(f"[LAND] Altitudine: {alt:.2f} m")

            if alt is not None and alt < 0.1:
                print("[LAND] Altitudine mică → dezarmez")
                self.vehicle._master.mav.command_long_send(
                    self.vehicle._master.target_system,
                    self.vehicle._master.target_component,
                    mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
                    0, 0, 21196, 0, 0, 0, 0, 0
                )
                break
            time.sleep(1)

        print("[DroneKit] Aterizare completă.")
        return "Drone Landing"
# Adaugă o comandă în coada de execuție pentru dronă.
# Comenzile pot fi: takeoff, land, orbit, auto_search etc.
# Vor fi procesate de state machine în ordinea primirii.

    def enqueue_command(self, command, args=None):
        self.command_queue.put((command, args or {}))

# Rulează un state machine pentru dronă care procesează comenzile primite din coadă (`command_queue`).
# Fiecare comandă este tratată secvențial: takeoff → zboară → revino → aterizează etc.
# Se ocupă de logica de zbor autonom și controlează tranziția între stările dronei (IDLE, TAKING_OFF, IN_AIR, LANDING).

    def _run_state_machine(self):
        # Pas 0: Conectare obligatorie
        self._connect_drone()
        if not self.connected:
            print("[STATE] Nu se poate porni state machine: drona nu este conectată.")
            return

        while True:
            command, args = self.command_queue.get()
            self.current_command = command

            if command == "land":
                self.current_state = "LANDING"
                stop_takeoff_event.set()
                self.land_drone()
                self.current_state = "IDLE"
                continue

            if command == "takeoff":
                self.current_state = "TAKING_OFF"
                alt = args.get("altitude", 2)
                mode = args.get("mode", "GUIDED")
                self.arm_and_takeoff(alt, mode)
                
                self.current_state = "IN_AIR"
                continue

            if self.current_state != "IN_AIR":
                print(f"[STATE] Ignor comanda '{command}'  pentru ca drona nu a decolat.")
                continue

            if command == "auto_search":
                self._handle_auto_search(args)
            elif command == "goto_and_return":
                self._handle_goto_and_return(args)
            elif command == "orbit":
                self._handle_orbit(args)

            self.current_command = None


# Returnează un snapshot cu starea curentă a dronei: poziție, altitudine, mod, stare armare și conexiune.
    def get_status(self):
        return {
            "state": self.current_state,
            "command": self.current_command,
            "alt": self.vehicle.location.global_relative_frame.alt if self.vehicle else None,
            "lat": self.location.lat,
            "lon": self.location.lon,
            "mode": self.vehicle.mode.name if self.vehicle else None,
            "armed": self.vehicle.armed if self.vehicle else None,
            "connected": self.connected
        }

    def close(self):
        if self.vehicle:
            print("[DroneKit] Închidere conexiune cu drona...")
            self.vehicle.close()
    def set_roi(self,location):
        msg = self.vehicle.message_factory.command_long_encode(
            0, 0,
            mavutil.mavlink.MAV_CMD_DO_SET_ROI,
            0,
            0, 0, 0, 0,
            location.lat, location.lon, location.alt
        )
        self.vehicle.send_mavlink(msg)

    def send_ned_velocity(self, velocity_x, velocity_y, velocity_z, duration):

        msg = self.vehicle.message_factory.set_position_target_local_ned_encode(
            0, 0, 0,
            mavutil.mavlink.MAV_FRAME_BODY_NED,
            0b0000111111000111,
            0, 0, 0,
            velocity_x, velocity_y, velocity_z,
            0, 0, 0,
            0, 0
        )
        for _ in range(duration):
            self.vehicle.send_mavlink(msg)
            time.sleep(1)

    #----Comenzi drona----
    
    # Trimite drona să orbiteze în jurul unei locații date (ex: locația unei detecții).
    # Se mișcă în cerc pe baza calculelor NED (velocity în North-East-Down).
    def _handle_orbit(self, args):
        
        center_location = args.get("location")
        radius = args.get("radius", 5)
        velocity = args.get("speed", 1.0)
        duration = args.get("duration", 20)
    
        if not (center_location and radius > 0 and velocity > 0 and duration > 0):
            print("[ORBIT] Parametri invalizi. Abort.")
            return
    
        print("[DRONA] Incep orbitarea in jurul punctului...")
        self.set_roi(center_location)
        try:
            self.vehicle.flush()
        except Exception as e:
            print(f"[WARN] flush() failed: {e}")
    
        angular_velocity = velocity / radius 
        step_time = 0.2 
        steps = int(duration / step_time)
    
        angle = 0
        start_time = time.monotonic()
    
        for step in range(steps):
            if self.vehicle.mode.name != "GUIDED":
                print("[ORBIT] Mod schimbat. Oprire orbita.")
                break
    
            vx = -velocity * math.sin(angle)
            vy = velocity * math.cos(angle)
    
            self.send_ned_velocity(vx, vy, 0, 1)
            try:
                self.vehicle.flush()
            except Exception as e:
                print(f"[WARN] flush() failed at step {step}: {e}")
    
            angle += angular_velocity * step_time
            elapsed = time.monotonic() - start_time
            if elapsed > duration:
                break
    
            time.sleep(step_time)
    
        print("[DRONA] Orbita completa/ intrerupta.")


    # Trimite drona la o locație dată, așteaptă, apoi se întoarce la punctul de decolare.
    def _handle_goto_and_return(self, args):


        target_location = args.get("location")
        speed = args.get("speed", 4)
        if target_location and speed:

   
            home_location = self.vehicle.location.global_relative_frame
            self.vehicle.groundspeed = speed

            print("[DRONA] Zbor către punctul țintă...")
            self.vehicle.simple_goto(target_location)

            while self.get_distance_metres(self.vehicle.location.global_relative_frame, target_location) > 2:
                print("[DRONA] Apropiere de punctul țintă...")
                time.sleep(1)

            print("[DRONA] Ajuns la destinație. Aștept 3 secunde...")
            time.sleep(3)

            print("[DRONA] Revenire la punctul inițial...")
            self.vehicle.simple_goto(home_location)
            while self.get_distance_metres(self.vehicle.location.global_relative_frame, home_location) > 2:
                print("[DRONA] Apropiere de poziția inițială...")
                time.sleep(1)

            print("[DRONA] Revenit la poziția inițială.")


    def get_distance_metres(self, aLocation1, aLocation2):

        dlat = aLocation2.lat - aLocation1.lat
        dlon = aLocation2.lon - aLocation1.lon
        return math.sqrt((dlat*dlat) + (dlon*dlon)) * 1.113195e5

    # Execută o căutare automată pe o zonă definită (serpuita), oprindu-se dacă se detectează un pericol.
    # În caz de detecție, zboară acolo, orbitează pentru asigurarea alarmei, activează servomotorul, apoi revine la bază.

    def _handle_auto_search(self, args):
        global detected_flag, event_location

        if self.vehicle.mode.name != "GUIDED":
            print("[AUTO] Modul actual nu este GUIDED. Abort misiune.")
            return

        area_size = args.get("area_size", 5)
        step = args.get("step", 1)
        height = args.get("height", 2)
        speed = args.get("speed", 4)

        if not all([area_size, step, height, speed]):
            print("[AUTO] Parametri insuficienti. Abort.")
            return

        print("[DRONA] Execut misiune...")

        home = LocationGlobalRelative(
            self.vehicle.location.global_relative_frame.lat,
            self.vehicle.location.global_relative_frame.lon,
            self.vehicle.location.global_relative_frame.alt
        )
        self.vehicle.groundspeed = speed
        print(f"[AUTO] Locatie de start: ({home.lat:.6f}, {home.lon:.6f})")

        dx = np.arange(0, area_size + step, step)
        dy = np.arange(0, area_size + step, step)

        print("[AUTO] Incep serpuirea...")

        def go_and_wait(target, label="destinatie", timeout=15):
            self.vehicle.simple_goto(target)
            try:
                self.vehicle.flush()
            except Exception as e:
                print(f"[WARN] flush() failed: {e}")

            start = time.time()
            while True:
                if self.vehicle.mode.name != "GUIDED":
                    print(f"[AUTO] Modul schimbat în {self.vehicle.mode.name}. Oprire misiune.")
                    return False

                dist = self.get_distance_metres(self.vehicle.location.global_relative_frame, target)
                print(f"[AUTO] Distanta pana la {label}: {dist:.2f} m")

                if dist <= 2:
                    print(f"[AUTO] Ajuns la {label}.")
                    return True

                if time.time() - start > timeout:
                    print(f"[AUTO] Timeout la {label}.")
                    return False

                time.sleep(0.3)

        for i, y in enumerate(dy):
            for x in (dx if i % 2 == 0 else reversed(dx)):

                if self.vehicle.mode.name != "GUIDED":
                    print(f"[AUTO] ⚠Modul schimbat în {self.vehicle.mode.name}. Intrerup misiunea.")
                    return

                if detected_flag and event_location:
                    print("[AUTO] Detectie activata! Deplasare la locația salvata.")
                    go_and_wait(event_location, "locatia de detectie", timeout=10)

                    print("[AUTO] Orbitare...")
                    self._handle_orbit({
                        "location": event_location,
                        "radius": 3,
                        "speed": 1.0,
                        "duration": 20
                    })

                    print("[AUTO] Activez servomotorul!")
                    activate_servos()

                    print("[AUTO] Revenire la baza...")
                    go_and_wait(home, "baza", timeout=20)

                    print("[AUTO] Misiune completa.")
                    return

                new_location = LocationGlobalRelative(
                    home.lat + (y / 111111),
                    home.lon + (x / (111111 * math.cos(math.radians(home.lat)))),
                    height
                )
                print(f"[AUTO] Merg la punctul ({x},{y}) → ({new_location.lat:.6f}, {new_location.lon:.6f})")
                go_and_wait(new_location, f"punct ({x},{y})", timeout=10)

        print("[AUTO] Misiune completă fara detectie. Revenire la baza...")
        go_and_wait(home, "bază", timeout=20)
        print("[AUTO] Misiune încheiata.")


#########################################################################
#########################################################################
# Initialize GPS provider
gps_provider = MockGPSProvider() if USE_SIMULATOR else DroneKitGPSProvider(bypass=False)
##########################################################################
##########################################################################

# Capturează frame-uri de la cameră și le salvează împreună cu datele GPS curente într-un buffer global.
# Rulează constant și este sursa de date pentru toate celelalte threaduri de detecție sau streaming.

def camera_thread():
    global frame_buffer
    logging.info("Firul principal (camera) a pornit.")
    while True:
        frame = picam2.capture_array()
        gps = gps_provider.get_location()
        gps_snapshot = {
            "lat": gps.lat,
            "lon": gps.lon,
            "alt": gps.alt,
            "timestamp": time.time()
        }
        if gps.lat and gps.lon:
            cv2.putText(frame, f"Lat: {gps.lat:.6f} Lon: {gps.lon:.6f} Alt: {gps.alt:.1f}", (10, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255,255,255), 2)
        with frame_lock:
            frame_buffer = {"image": frame.copy(), "gps": gps_snapshot}
        time.sleep(0.01)
        # logging.info("Firul camera_thread este activ...................")  

# Rulează modelul YOLOv11 pe frame-urile capturate pentru a detecta obiecte, inclusiv „om_la_inec”.
# În caz de detecție, salvează poziția GPS și trimite informații prin WebSocket.
# Marchează pe imagine offsetul și distanța față de centrul camerei.
def yolo_function_thread():
    global frame_buffer, yolo_output_frame, detected_flag, popup_sent, last_detection_time, frame_counter, event_location
    cam_x, cam_y = 320, 240
    PIXELS_PER_CM = 10
    object_present = False
    model = YOLO("my_model.pt")
    
    while not stop_detection_event.is_set():
    
        if not streaming:
            time.sleep(0.1)
            continue
        
        frame_counter += 1
        if frame_counter % detection_frame_skip != 0:
            time.sleep(0.01)
            continue

        logging.info("Firul yolo_function_thread este activ...................")    
        with frame_lock:
            data = frame_buffer.copy() if frame_buffer is not None else None
        if data is None:
            time.sleep(0.05)
            continue
            
        frame = data["image"]
        gps_info = data["gps"]
        results = model(frame, verbose=False)
        annotated = frame.copy()
        names = results[0].names
        class_ids = results[0].boxes.cls.tolist()
        boxes = results[0].boxes.xyxy.cpu().numpy()
        
        if gps_info["lat"] and gps_info["lon"]:
            gps_text = f"Lat: {gps_info['lat']:.6f} Lon: {gps_info['lon']:.6f} Alt: {gps_info['alt']:.1f}"
            timestamp_text = f"Timp: {time.strftime('%H:%M:%S', time.localtime(gps_info['timestamp']))}"
            cv2.putText(annotated, gps_text, (10, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0,255,255), 2)
            cv2.putText(annotated, timestamp_text, (10, 45), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0,255,255), 2)
            
        current_detection = False
        for i, cls_id in enumerate(class_ids):
            x1, y1, x2, y2 = boxes[i].astype(int)
            label = names[int(cls_id)]
            color = (0, 255, 0) if label == "om_la_inec" else (255, 0, 0)
            cv2.rectangle(annotated, (x1, y1), (x2, y2), color, 2)
            cv2.putText(annotated, label, (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)
            
            if label == "om_la_inec":
                current_detection = True
                if not object_present:
                    # Save event location when first detected
                    if gps_info["lat"] and gps_info["lon"]:
                        event_location = LocationGlobalRelative(
                            gps_info["lat"], 
                            gps_info["lon"], 
                            gps_info["alt"]
                        )
                        logging.info(f"[DETECTION] Event location saved: {event_location}")
                    
                    detected_flag = True
                    popup_sent = True
                    last_detection_time = time.time()
                    object_present = True
                    
                obj_x = (x1 + x2) // 2
                obj_y = (y1 + y2) // 2
                dx_cm = (obj_x - cam_x) / PIXELS_PER_CM
                dy_cm = (obj_y - cam_y) / PIXELS_PER_CM
                dist_cm = (dx_cm**2 + dy_cm**2)**0.5
                cv2.line(annotated, (cam_x, cam_y), (obj_x, obj_y), (0, 0, 255), 2)
                cv2.circle(annotated, (cam_x, cam_y), 5, (255, 0, 0), -1)
                cv2.circle(annotated, (obj_x, obj_y), 5, (0, 255, 0), -1)
                offset_text = f"x:{dx_cm:.1f}cm | y:{dy_cm:.1f}cm"
                dist_text = f"Dist: {dist_cm:.1f}cm"
                cv2.putText(annotated, offset_text, (20, 70), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0,255,255), 2)
                cv2.putText(annotated, dist_text, (20, 100), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0,255,255), 2)
                
        if not current_detection:
            detected_flag = False
            popup_sent = False
            object_present = False
        
        obiecte_detectate = []
        nivel_detectat = None

        for i, cls_id in enumerate(class_ids):
            label = names[int(cls_id)]

            if label in ["Meduze", "Rechin", "person", "rip_current"]:
                obiecte_detectate.append(label)

            if label == "lvl_mic":
                nivel_detectat = "mic"
            elif label == "lvl_mediu":
                nivel_detectat = "mediu"
            elif label == "lvl_adanc":
                nivel_detectat = "adânc"

        # Trimite doar dacă există modificări (poți adăuga și comparație cu stare anterioară dacă vrei)
        socketio.emit("detection_update", {
            "obiecte": obiecte_detectate,
            "nivel": nivel_detectat
        })     
        with output_lock:
            yolo_output_frame = cv2.imencode('.jpg', annotated)[1].tobytes()
        time.sleep(0.01)
        
# Trimite constant frame-ul brut capturat de cameră către interfața web.
# Scrie rezultatul JPEG în `output_frame` pentru ruta `/video_feed`.
def stream_thread():
    global output_frame
    logging.info("Firul 3 (livrare raw) a pornit.")
    while True:
        if not streaming:
            time.sleep(0.1)
            continue
        with frame_lock:
            data = frame_buffer.copy() if frame_buffer else None
        if data is None:
            time.sleep(0.05)
            continue
        # print("Firul stream_thread rulează...")
        
        jpeg = cv2.imencode('.jpg', data["image"])[1].tobytes()
        with output_lock:
            output_frame = jpeg
        time.sleep(0.05)


# Rulează modelul YOLO pentru detecția de forme de viață: meduze, rechini și oameni.
# Dacă este activat modul smart, schimbă automat streamul cu modul de clasificare înec (`xgb`) când detectează o persoană.
# În orice caz, trimite etichetele detectate prin WebSocket și le desenează în streamul `mar_feed`.

def livings_inference_thread(video=None):

    global mar_output_frame, frame_buffer
    obiecte_detectate = []
    model = YOLO("models/livings.pt")
    while not stop_detection_liv_event.is_set():
        logging.info("Firul livings_inference_thread rulează...")
        obiecte_detectate.clear()
        if not streaming:
            time.sleep(0.1)
            continue
            
        with frame_lock:
            data = frame_buffer.copy() if frame_buffer is not None else None
        if data is None:
            time.sleep(0.05)
            continue
            
        frame = data["image"]
        gps_info = data["gps"]
        
        results = model.predict(source=frame, conf=0.4, stream=True)
        
        
        for r in results:
            boxes = r.boxes
            for box in boxes:
                cls_id = int(box.cls[0])
                conf = float(box.conf[0])
                
                if 0 <= cls_id < 3:
                    name = ["Meduze", "Rechin", "person"][cls_id]
                else:
                    print(f"[WARN] cls_id invalid {cls_id} -> ignorăm")
                    continue
                
                label = f"{name} {conf:.2f}"
                obiecte_detectate.append(name)
                x1, y1, x2, y2 = map(int, box.xyxy[0])
                cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 255), 2)
                cv2.putText(frame, label, (x1, y1 - 10),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)

        if smart_stream_mode:
            for r in results:
                for box in r.boxes:
                    cls_id = int(box.cls[0])
                    if cls_id == 2:  # person
                        print("[SMART SWITCH] Detected person → switching to raw + xgb")
                        stop_detection_liv_event.set()
                        stop_segmentation_event.set()

                        socketio.emit("stream_config_update", {"left": "raw", "right": "xgb"})

                        global pose_thread, stop_pose_event
                        if pose_thread is None or not pose_thread.is_alive():
                            stop_pose_event.clear()
                            pose_thread = start_thread(pose_xgb_inference_thread, "PoseXGBDetection")
                        return        


        socketio.emit("detection_update", {"obiecte": obiecte_detectate})
    
        with mar_lock:
            if obiecte_detectate:
                mar_output_frame = cv2.imencode('.jpg', frame)[1].tobytes()
        time.sleep(0.01)

# Rulează modelul YOLOv11 de segmentare semantică pentru a colora zonele din apă pe baza adâncimii sau a curenților de rupere.
# Suportă clase ca: „lvl_mic”, „lvl_mediu”, „lvl_adanc”, „rip_current”.
# Fiecare mască este desenată peste imaginea originală și este transmisă prin streamul `seg_feed`.
# Trimite și eticheta corespunzătoare prin WebSocket la frontend.

def segmentation_inference_thread(video=None):
    global seg_output_frame

    logging.info("Firul segmentation_inference_thread rulează...")
    model = YOLO("models/yolo11n-seg-custom.pt")  # <- model YOLOv11n SEGMENTARE

    # Culori personalizate pentru fiecare clasă
    color_map = {
        "lvl_mic": (255, 255, 0),
        "lvl_mediu": (0, 255, 0),
        "lvl_adanc": (0, 0, 255),
        "rip_current": (0, 165, 255)
    }

    while not stop_segmentation_event.is_set():
        if not streaming:
            time.sleep(0.1)
            continue

        with frame_lock:
            data = frame_buffer.copy() if frame_buffer is not None else None
        if data is None:
            time.sleep(0.05)
            continue

        frame = data["image"]
        gps_info = data["gps"]
        annotated = frame.copy()
        nivel_detectat = None

        # rulează modelul YOLOv11n pe frame
        results = model.predict(source=frame, conf=0.5, stream=False)

        for r in results:
            masks = r.masks
            names = r.names

            if masks is not None and masks.data is not None:
                for i, mask in enumerate(masks.data):
                    if i >= len(r.boxes.cls):
                        continue  # siguranță la indexare

                    cls_id = int(r.boxes.cls[i].item())
                    label = names[cls_id]
                    color = color_map.get(label, (0, 255, 255))

                    mask_np = mask.cpu().numpy()
                    annotated[mask_np > 0.5] = color

                    # Afișează eticheta în centrul măștii
                    ys, xs = np.where(mask_np > 0.5)
                    if len(xs) > 0 and len(ys) > 0:
                        cx, cy = int(np.mean(xs)), int(np.mean(ys))
                        cv2.putText(annotated, label, (cx, cy),
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 0), 2, lineType=cv2.LINE_AA)

                    # Interpretare nivel
                    if label in ["lvl_mic", "lvl_mediu", "lvl_adanc"]:
                        nivel_detectat = label.replace("lvl_", "")
                    elif label == "rip_current":
                        nivel_detectat = "rip_current"

        socketio.emit("detection_update", {"nivel": nivel_detectat})

        with seg_lock:
            seg_output_frame = cv2.imencode('.jpg', annotated)[1].tobytes()

        time.sleep(0.05)

# Aplică YOLO Pose Estimation pe fiecare frame și extrage 34 de coordonate per frame (x, y).
# Păstrează un buffer de 30 frame-uri consecutive, le transformă într-un vector [1020] și face clasificare cu XGBoost.
# Dacă este detectat „inec”, trimite alertă prin WebSocket și marchează frame-ul.
# Dacă nu apare niciun „inec” timp de 10 secunde în modul smart, revine automat la streamul inițial `mar + seg`.
def pose_xgb_inference_thread(video=None):
    global pose_output_frame
    global pose_output_frame
    last_inec_time = time.time() 

    POSE_CONNECTIONS = [
    (0, 1), (0, 2),       # nose → eyes
    (1, 3), (2, 4),       # eyes → ears
    (5, 6),               # shoulders
    (5, 7), (7, 9),       # left arm
    (6, 8), (8,10),       # right arm
    (5,11), (6,12),       # shoulders → hips
    (11,12),              # hips
    (11,13), (13,15),     # left leg
    (12,14), (14,16)      # right leg
]

    model = YOLO("models/yolo11n-pose.pt")
    buffer = deque(maxlen=30)

    while not stop_pose_event.is_set():
        logging.info("Firul pose_xgb_inference_thread rulează...")

        if not streaming:
            time.sleep(0.1)
            continue

        with frame_lock:
            data = frame_buffer.copy() if frame_buffer is not None else None
        if data is None:
            time.sleep(0.05)
            continue

        frame = data["image"]
        gps_info = data["gps"]
        timestamp = time.time()

        results = model.predict(frame, stream=True)

        keypoints = None
        for result in results:
            if result.keypoints is not None and len(result.keypoints.xy) > 0:
                if result.keypoints is not None and result.keypoints.xy is not None:
                    keypoints_tensor = result.keypoints.xy
                    confs_tensor = result.keypoints.conf

                    if len(keypoints_tensor) > 0 and confs_tensor is not None:
                        keypoints = keypoints_tensor[0].cpu().numpy()
                        confs = confs_tensor[0].cpu().numpy()
                    else:
                        continue
                else:
                    continue

        if keypoints is None or keypoints.shape != (17, 2):
            continue

        vec34 = keypoints.flatten()
        buffer.append(vec34)

        # Desenare keypoints
        for i, (x, y) in enumerate(keypoints):
            if confs[i] > 0.4:
                cv2.circle(frame, (int(x), int(y)), 3, (0, 255, 0), -1)
        # Desenare conexiuni
        for i1, i2 in POSE_CONNECTIONS:
            if confs[i1] > 0.4 and confs[i2] > 0.4:
                x1, y1 = keypoints[i1]
                x2, y2 = keypoints[i2]
                cv2.line(frame, (int(x1), int(y1)), (int(x2), int(y2)), (255, 0, 0), 2)


        if len(buffer) == 30:
            vector_1020 = np.array(buffer).flatten()
            prediction = xgb_predict(vector_1020)

            if prediction == "inec":

                cv2.putText(frame, "POSIBIL INEC!", (20, 60), cv2.FONT_HERSHEY_SIMPLEX, 1.2, (0, 0, 255), 3)

                lat = gps_info.get('lat')
                lon = gps_info.get('lon')

                lat_str = f"{lat:.6f}" if lat is not None else "necunoscut"
                lon_str = f"{lon:.6f}" if lon is not None else "necunoscut"

                logging.warning(f"[ALERTĂ INEC] Timp: {timestamp:.2f} | Lat: {lat_str} | Lon: {lon_str}")

                socketio.emit("detection_update", {
                    "eveniment": "POSIBIL INEC!",
                    "nivel": "inec",
                    "timestamp": timestamp,
                    "gps": {
                        "lat": lat_str,
                        "lon": lon_str
                    }}) 

        if smart_stream_mode and time.time() - last_inec_time > 10:
            print("[SMART SWITCH] No inec in 10s → switching back to mar + seg")
            stop_pose_event.set()
            global detection_liv_thread, segmnetation_thread
            stop_detection_liv_event.clear()
            stop_segmentation_event.clear()
            start_thread(livings_inference_thread, "LivingsThread")
            start_thread(segmentation_inference_thread, "SegmentationThread")
            socketio.emit("stream_config_update", {"left": "mar", "right": "seg"})
            return
           

        with pose_lock:
            pose_output_frame = cv2.imencode('.jpg', frame)[1].tobytes()

        time.sleep(0.05)


# === Flask Routes ===
@app.route("/")
def index():
    return render_template("index.html")
# Trimite streamul video brut de la cameră către browser, tip MJPEG.
@app.route("/video_feed")
def video_feed():
    def generate():
        global output_frame
        while True:
            if not streaming:
                time.sleep(0.1)
                continue
            with output_lock:
                frame = output_frame if output_frame is not None else blank_frame()
            yield (b"--frame\r\nContent-Type: image/jpeg\r\n\r\n" + frame + b"\r\n")
            time.sleep(0.05)
    return Response(generate(), mimetype="multipart/x-mixed-replace; boundary=frame")

@app.route("/yolo_feed")
def yolo_feed():
    def generate():
        global yolo_output_frame
        while True:
            if not streaming:
                time.sleep(0.1)
                continue
            with output_lock:
                frame = yolo_output_frame if yolo_output_frame is not None else blank_frame()
            yield (b"--frame\r\nContent-Type: image/jpeg\r\n\r\n" + frame + b"\r\n")
            time.sleep(0.05)
    return Response(generate(), mimetype="multipart/x-mixed-replace; boundary=frame")

@app.route("/yolo_feed_snapshot")
def yolo_feed_snapshot():
    global yolo_output_frame
    with output_lock:
        frame = yolo_output_frame if yolo_output_frame is not None else blank_frame()
    return Response(frame, mimetype='image/jpeg')



@app.route("/mar_feed")
def mar_feed():
    def generate():
        while True:
            with mar_lock:
                frame = mar_output_frame or blank_frame()
            yield (b"--frame\r\nContent-Type: image/jpeg\r\n\r\n" + frame + b"\r\n")
            time.sleep(0.05)
    return Response(generate(), mimetype='multipart/x-mixed-replace; boundary=frame')

@app.route("/seg_feed")
def seg_feed():
    def generate():
        while True:
            with seg_lock:
                frame = seg_output_frame or blank_frame()
            yield (b"--frame\r\nContent-Type: image/jpeg\r\n\r\n" + frame + b"\r\n")
            time.sleep(0.05)
    return Response(generate(), mimetype='multipart/x-mixed-replace; boundary=frame')

@app.route("/xgb_feed")
def xgb_feed():
    def generate():
        while True:
            with pose_lock:
                frame = pose_output_frame or blank_frame()
            yield (b"--frame\r\nContent-Type: image/jpeg\r\n\r\n" + frame + b"\r\n")
            time.sleep(0.05)
    return Response(generate(), mimetype='multipart/x-mixed-replace; boundary=frame')

@app.route("/set_right_stream", methods=["POST"])
def set_right_stream():
    global right_stream_type,detection_thread,detection_liv_thread,stop_detection_event,stop_detection_liv_event,pose_thread,stop_pose_event,segmnetation_thread,stop_segmentation_event
    data = request.json
    selected = data.get("type")
    logging.info(f"[FLASK] Set right stream type: {selected}")
    if selected in ["yolo", "seg", "mar", "xgb"]:
        right_stream_type = selected
        logging.info(f"[FLASK] right_stream_type: {right_stream_type}")
        if selected == "yolo":
            #oprire alte threaduri
            stop_detection_liv_event.set()
            stop_segmentation_event.set()
            stop_pose_event.set()
             #pornire thread
            if detection_thread is None or not detection_thread.is_alive():
                stop_detection_event.clear()
                detection_thread =start_thread(yolo_function_thread, "DetectionThread")
            #repornire thread
            if detection_thread and detection_thread.is_alive():
                stop_detection_event.set()
                detection_thread.join()  # așteaptă să se termine curentul thread
                stop_detection_event.clear()
                detection_thread =start_thread(yolo_function_thread, "DetectionThread")    

        elif selected == "seg":
            #oprire alte threaduri
            stop_detection_event.set()
            stop_detection_liv_event.set()
            stop_pose_event.set()
            #pornire thread
            if segmnetation_thread is None or not segmnetation_thread.is_alive():
                stop_segmentation_event.clear()
                segmnetation_thread =start_thread(segmentation_inference_thread, "SegmentationDetection")
            #repornire thread
            if segmnetation_thread and segmnetation_thread.is_alive():
                stop_segmentation_event.set()
                segmnetation_thread.join()  # așteaptă să se termine curentul thread
                stop_segmentation_event.clear()
                segmnetation_thread =start_thread(segmentation_inference_thread, "SegmentationDetection") 

        elif selected == "mar":
            #oprire alte threaduri
            stop_detection_event.set()
            stop_segmentation_event.set()
            stop_pose_event.set()
            #pornire thread
            if detection_liv_thread is None or not detection_liv_thread.is_alive():
                stop_detection_liv_event.clear()
                detection_liv_thread =start_thread(livings_inference_thread, "LivingsDetection")
            #repornire thread
            if detection_liv_thread and detection_liv_thread.is_alive():
                stop_detection_liv_event.set()
                detection_liv_thread.join()  # așteaptă să se termine curentul thread
                stop_detection_liv_event.clear()
                detection_liv_thread =start_thread(livings_inference_thread, "LivingsDetection") 
        elif selected == "xgb":
            #oprire alte threaduri
            stop_detection_event.set()
            stop_detection_liv_event.set()
            stop_segmentation_event.set()
            #pornire thread
            if pose_thread is None or not pose_thread.is_alive():
                stop_pose_event.clear()
                pose_thread =start_thread(pose_xgb_inference_thread, "PoseXGBDetection")
            #repornire thread
            if pose_thread and pose_thread.is_alive():
                stop_pose_event.set()
                pose_thread.join()  # așteaptă să se termine curentul thread
                stop_pose_event.clear()
                pose_thread =start_thread(pose_xgb_inference_thread, "PoseXGBDetection") 
        else:
            logging.error(f"[FLASK] Invalid stream type: {selected}")



        return jsonify({"status": "ok", "current": right_stream_type})
    return jsonify({"status": "invalid"})
    
# Trimite streamul ales (YOLO, segmentare etc.) către interfață.
# Se schimbă în funcție de `right_stream_type`.
@app.route("/right_feed")
def right_feed():
    def generate():
        global right_stream_type,detection_thread,detection_liv_thread 

        while True:
            if right_stream_type == "yolo":
                with output_lock:
                    frame = yolo_output_frame or blank_frame()
            elif right_stream_type == "seg":

                with seg_lock:
                    frame = seg_output_frame or blank_frame()
            elif right_stream_type == "mar":

                with mar_lock:
                    frame = mar_output_frame or blank_frame()
            elif right_stream_type == "xgb":
                with pose_lock:
                    frame = pose_output_frame or blank_frame()
            else:
                frame = blank_frame()

            yield (b"--frame\r\nContent-Type: image/jpeg\r\n\r\n" + frame + b"\r\n")
            time.sleep(0.05)
    return Response(generate(), mimetype="multipart/x-mixed-replace; boundary=frame")
# Pornește threadurile `seg` + `mar`, și pornește clasificarea înec (`xgb`) doar când se activează `pose_triggered`.
# Garantează că inferența complexă începe doar dacă a fost activată explicit.
@app.route("/start_official")
def start_official():
    global pose_thread_started
    start_thread(livings_inference_thread, "LivingsDetection")
    start_thread(segmentation_inference_thread, "SegmentationDetection")

    def watch_pose_trigger():
        global pose_triggered, pose_thread_started
        while not pose_thread_started:
            if pose_triggered:
                pose_thread_started = True
                start_thread(pose_xgb_inference_thread, "PoseXGBDetection")
                break
            time.sleep(0.5)

    start_thread(watch_pose_trigger, "Watcher_XGB")
    return jsonify({"status": "official detection started"})

@app.route("/start_stream")
def start_stream():
    global streaming
    streaming = True
    return jsonify({"status": "started"})

@app.route("/stop_stream")
def stop_stream():
    global streaming
    streaming = False
    stop_detection_event.set()
    stop_detection_liv_event.set()
    return jsonify({"status": "stopped"})

@app.route("/detection_status")
def detection_status():
    return jsonify({"detected": popup_sent})
    
# Activează manual servomotoarele printr-un request GET.
@app.route("/misca")
def activate():
    print("Comanda actionare servomotoare")
    activate_servos()
    return "Servomotor activat"
  





@app.route("/start_seg")
def start_seg():
    start_thread(lambda: segmentation_inference_thread(), "SegThread")
    return jsonify({"status": "segmentation started"})

@app.route("/start_livings")
def start_livings():
    start_thread(lambda: livings_inference_thread(), "LivingsThread")
    return jsonify({"status": "livings started"})

@app.route("/start_pose_xgb")
def start_pose_xgb():
    start_thread(lambda: pose_xgb_inference_thread(), "XGBThread")
    return jsonify({"status": "xgb started"})

# Activează modul „smart stream” – dacă detectează „person” în `livings` și comută automat la `pose_xgb`.
# Când nu se mai detectează înec, revine la `mar + seg`.        
@app.route("/start_smart_mode")
def start_smart_mode():
    global smart_stream_mode
    smart_stream_mode = True
    socketio.emit("stream_config_update", {"left": "mar", "right": "seg"})

    stop_detection_event.set()
    stop_pose_event.set()

    stop_detection_liv_event.clear()
    stop_segmentation_event.clear()

    start_thread(segmentation_inference_thread, "SegmentationThread")
    start_thread(livings_inference_thread, "LivingsThread")

    return jsonify({"status": "smart stream mode activated"})


@socketio.on('drone_command')
def handle_drone_command(data):
    global event_location
    action = data.get('action')
    print(f"[WS] Comandă primită: {action}")

    if action == 'takeoff':
        gps_provider.enqueue_command("takeoff", {"altitude": 2, "mode": "GUIDED"})
    elif action == 'land':
       gps_provider.enqueue_command("land")
    elif action == 'goto_and_return':
       gps_provider.enqueue_command("goto_and_return", {"location": event_location, "speed": 4})
    elif action == 'orbit':
        gps_provider.enqueue_command("orbit", {"location": event_location, "radius": 5, "speed": 1.0, "duration": 20})
    elif action == 'auto_search':
        gps_provider.enqueue_command("auto_search", {"area_size": 5, "step": 1, "height": 2, "speed": 4})
    elif action == 'mission':
          gps_provider.enqueue_command("takeoff", {"altitude": 2, "mode": "GUIDED"})
          gps_provider.enqueue_command("auto_search", {"area_size": 5, "step": 1, "height": 2, "speed": 4})
          gps_provider.enqueue_command("land")      
    else:
        print(f"[WS] Comandă necunoscută: {action}")
        
@socketio.on('joystick_command')
def handle_joystick(data):
    x = data.get('x', 0)
    y = data.get('y', 0)
    z = data.get('z', 0)
    yaw = data.get('yaw', 0)

    print(f"[JOYSTICK] x={x:.2f} y={y:.2f} z={z:.2f} yaw={yaw:.2f}")
    try:
        gps_provider.ensure_connection()
        vehicle = gps_provider.vehicle

        # Aici poți adapta comenzile – ex:
        send_ned_velocity(x, y, z, 1, vehicle)
    except Exception as e:
        print(f"[JOYSTICK ERROR] {e}")
 # Trimite periodic (la fiecare secundă) prin WebSocket statusul actual al dronei: stare, mod, baterie, poziție, etc.       
def status_broadcast_loop():
    while True:
        try:
            if gps_provider.connected and gps_provider.vehicle:
                socketio.emit('drone_status', {
                    "connected": True,
                    "battery": {
                        "voltage": gps_provider.vehicle.battery.voltage,
                        "current": gps_provider.vehicle.battery.current
                    },
                    "armed": gps_provider.vehicle.armed,
                    "mode": gps_provider.vehicle.mode.name,
                    "location": {
                        "lat": gps_provider.vehicle.location.global_frame.lat,
                        "lon": gps_provider.vehicle.location.global_frame.lon
                    },
                    "event_location": {
                        "lat": event_location.lat if event_location else None,
                        "lon": event_location.lon if event_location else None
                    },
                    "altitude_global": gps_provider.vehicle.location.global_frame.alt if gps_provider.vehicle.location else None,
                    "altitude_relative": gps_provider.vehicle.location.global_relative_frame.alt if gps_provider.vehicle.location else None
                })
        except Exception as e:
            print(f"[STATUS ERROR] {e}")
        time.sleep(1)

start_thread(camera_thread, "CameraThread")
start_thread(stream_thread, "StreamThread")
start_thread(status_broadcast_loop, "StatusBroadcast")

if __name__ == "__main__":


    logging.info("Pornire server Flask + SocketIO")
    socketio.run(app, host="0.0.0.0", port=5000)
