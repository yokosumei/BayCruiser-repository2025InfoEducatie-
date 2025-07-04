from flask import Flask, render_template, Response, request, jsonify
from flask_socketio import SocketIO
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
from pymavlink import mavutil

logging.basicConfig(level=logging.INFO, format='[%(levelname)s] (%(threadName)s) %(message)s')


app = Flask(__name__)
socketio = SocketIO(app, async_mode='threading')

app.config['UPLOAD_FOLDER'] = 'static/uploads'
os.makedirs(app.config['UPLOAD_FOLDER'], exist_ok=True)

model = YOLO("my_model.pt")
picam2 = Picamera2()
picam2.configure(picam2.create_video_configuration(main={"format": "RGB888", "size": (640, 480)}))
picam2.start()

GPIO.setmode(GPIO.BOARD)
GPIO.setup(11, GPIO.OUT)
GPIO.setup(12, GPIO.OUT)
servo1 = GPIO.PWM(11, 50)
servo2 = GPIO.PWM(12, 50)
servo1.start(7.5)
servo2.start(7.5)
time.sleep(0.3)
servo1.ChangeDutyCycle(0)
servo2.ChangeDutyCycle(0)

streaming = False
frame_lock = threading.Lock()
output_lock = threading.Lock()

frame_buffer = None
output_frame = None
yolo_output_frame = None
detected_flag = False
popup_sent = False
last_detection_time = 0
detection_frame_skip = 2
frame_counter = 0
event_location = None  # Fixed: Initialize event_location

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

def start_thread(func, name="WorkerThread"):
    """Helper function to start daemon threads"""
    t = threading.Thread(target=func, name=name, daemon=True)
    t.start()
    return t

def activate_servos():
    logging.debug("Activare servomotoare")
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
        
        # Start connection in background thread
        start_thread(self._connect_drone, "DroneConnection")

    def _connect_drone(self):
        """Connect to drone in background thread"""
        try:
            print("[DroneKitGPSProvider] Conectare la dronă...")
            self.vehicle = connect(self.connection_string, baud=self.baud_rate, wait_ready=False)
            time.sleep(1)
            self.vehicle.add_attribute_listener('location.global_frame', self.gps_callback)
            
            # if not self.bypass:
            #     self.vehicle.parameters['ARMING_CHECK'] = 0
            #     self.vehicle.parameters['EKF_CHECK_THRESH'] = 0.5
            
            time.sleep(5)
            self.connected = True
            logging.info("[DroneKitGPSProvider] Conectare la Pixhawk completă")
        except Exception as e:
            logging.error(f"[DroneKitGPSProvider] Eroare la conectare: {e}")
            self.connected = False

    def ensure_connection(self):
        """Ensure drone is connected before operations"""
        if not self.connected or self.vehicle is None:
            raise Exception("Drone not connected")
        return True

    def _wait_until_ready(self, timeout=30):
        if not self.ensure_connection():
            return False
            
        if self.bypass:
            print(" ByPASS=TRUE -> EKF OK:", self.vehicle.ekf_ok)
            print("  -> GPS fix:", self.vehicle.gps_0.fix_type)
            print("  -> Sateliți:", self.vehicle.gps_0.satellites_visible)
            print("  -> Sistem:", self.vehicle.system_status.state)
            print("[DroneKit] Bypass activ → simulăm dronă armabilă.")
            return True

        print("[DroneKit] Așteptăm ca drona să fie armabilă...")
        start = time.time()
        while not self.vehicle.is_armable:
            print("BYPASS=FALSE  -> EKF OK:", self.vehicle.ekf_ok)
            print("  -> GPS fix:", self.vehicle.gps_0.fix_type)
            print("  -> Sateliți:", self.vehicle.gps_0.satellites_visible)
            print("  -> Sistem:", self.vehicle.system_status.state)
            if time.time() - start > timeout:
                print("[DroneKit] Timeout atins. Nu e armabilă.")
                return False
            time.sleep(1)
        print("[DroneKit] Drona este gata.")
        return True
    
    def wait_until_ready(self, timeout=30):
        if not self.ensure_connection():
            return False
        

        print("[INFO] Trimit comanda de armare forțată (MAV_CMD_COMPONENT_ARM_DISARM)...")
        self.vehicle._master.mav.command_long_send(
            self.vehicle._master.target_system,
            self.vehicle._master.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            0,          # confirmation
            1,          # param1: 1=arm, 0=disarm
            21196,      # param2: magic code pentru override
            0, 0, 0, 0, 0
        )

        self.vehicle.parameters['FS_THR_ENABLE'] = 0
        self.vehicle.parameters['ARMING_CHECK'] = 0

        print("..............[DroneKit] Așteptăm ca drona să fie armabilă...")
        start = time.time()
        while not self.vehicle.armed:
            print("..  -> EKF OK:", self.vehicle.ekf_ok)
            print("..  -> GPS fix:", self.vehicle.gps_0.fix_type)
            print("..  -> Sateliți:", self.vehicle.gps_0.satellites_visible)
            print("..  -> Sistem:", self.vehicle.system_status.state)
            if time.time() - start > timeout:
                print(".....[DroneKit] Timeout atins. Nu e armabilă.")
                return False
            time.sleep(1)

             
        print("[DroneKit] Drona este gata.")
        return True

    def gps_callback(self, self_ref, attr_name, value):
        try:
            self.location = GPSValue(value.lat, value.lon, value.alt)
            logging.debug(f"[GPS] lat={value.lat}, lon={value.lon}, alt={value.alt}")
        except Exception as e:
            logging.exception("[GPS] Eroare în gps_callback")

    def get_location(self):
        return self.location
    
    
      

    def arm_and_takeoff(self, target_altitude,vehicle_mode):
        try:
            self.ensure_connection()
        except:
            return "[DroneKit] Drone not connected"
            
        if not self.wait_until_ready():
            return "[DroneKit] Nu e armabilă. Ieșire."
        # vehicle_mode=GUIDED,STABILIZE
        print("[DroneKit] Armare..........in mod ",vehicle_mode)
        self.vehicle.mode = VehicleMode(vehicle_mode)
        time.sleep(3)

        self.vehicle.armed = True

        while not self.vehicle.armed:
            print("  -> Așteptăm armarea...",vehicle_mode)
            print("Mode:", self.vehicle.mode.name)
            print("Is armable:", self.vehicle.is_armable)
            print("EKF OK:", self.vehicle.ekf_ok)
            print("System status:", self.vehicle.system_status.state)
            print("GPS fix:", self.vehicle.gps_0.fix_type)
            print("Satellites:", self.vehicle.gps_0.satellites_visible)
            time.sleep(1)

        if self.bypass:
            print("[DroneKit] Bypass activ → simulăm decolare.")
            return "Drone Takeoff (simulat)"

        print(f"[DroneKit] Decolare la {target_altitude}m...")
        self.vehicle.simple_takeoff(target_altitude)

        while True:
            alt = self.vehicle.location.global_relative_frame.alt
            print(f"  -> Altitudine curentă: {alt:.2f} m")
            if alt >= target_altitude * 0.95:
                print("[DroneKit] Altitudine atinsă.")
                break
            time.sleep(1)

        return "Drone Takeoff"
    
    
 def land_drone(self):
    if self.bypass:
        print("[DroneKit] Bypass activ → simulăm aterizare.")
        return "Drone Landing (simulat)"
        
    try:
        self.ensure_connection()
    except:
        return "[DroneKit] Drone not connected"
        


    print("[DroneKit] Aterizare...")
    self.vehicle.mode = VehicleMode("LAND")
    time.sleep(2)

    # Monitorizează altitudinea și așteaptă să ajungă aproape de sol
    while True:
        alt = self.vehicle.location.global_relative_frame.alt
        print(f"  -> Altitudine curentă: {alt:.2f} m")

        if alt is not None and alt < 0.2:
            print("[DroneKit] Altitudine joasă – forțăm dezarmarea...")
            try:
                # Comandă directă de dezarmare cu magic code
                self.vehicle._master.mav.command_long_send(
                    self.vehicle._master.target_system,
                    self.vehicle._master.target_component,
                    mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
                    0,          # confirmation
                    0,          # param1: 0 = disarm
                    21196,      # param2: magic disarm
                    0, 0, 0, 0, 0
                )
            except Exception as e:
                print("[DroneKit] Eroare la comanda de dezarmare:", e)
            break

        time.sleep(1)

    # Așteaptă să confirme că drona s-a dezarmat
    timeout = time.time() + 10  # maxim 10 secunde
    while self.vehicle.armed and time.time() < timeout:
        print("  -> Așteptăm dezarmarea completă...")
        time.sleep(1)

    if self.vehicle.armed:
        print("[WARN] Drona NU s-a dezarmat automat. Forțăm .armed = False")
        self.vehicle.armed = False

    print("[DroneKit] Aterizare completă.")
    return "Drone Landing"

    def _land_drone(self):
        try:
            self.ensure_connection()
        except:
            return "[DroneKit] Drone not connected"
            
        if self.bypass:
            print("[DroneKit] Bypass activ → simulăm aterizare.")
            return "Drone Landing (simulat)"

        print("[DroneKit] Aterizare...")
        self.vehicle.mode = VehicleMode("LAND")
        while self.vehicle.armed:
            print("  -> Așteptăm dezarmarea...")
            time.sleep(1)
        print("[DroneKit] Aterizare completă.")
        return "Drone Landing"

    def close(self):
        if self.vehicle:
            print("[DroneKit] Închidere conexiune cu drona...")
            self.vehicle.close()

# === Utility Functions ===
def set_roi(location, vehicle):
    """Set Region of Interest for camera gimbal"""
    msg = vehicle.message_factory.command_long_encode(
        0, 0,
        mavutil.mavlink.MAV_CMD_DO_SET_ROI,
        0,
        0, 0, 0, 0,
        location.lat, location.lon, location.alt
    )
    vehicle.send_mavlink(msg)

def send_ned_velocity(velocity_x, velocity_y, velocity_z, duration, vehicle):
    """Send NED velocity commands to drone"""
    msg = vehicle.message_factory.set_position_target_local_ned_encode(
        0, 0, 0,
        mavutil.mavlink.MAV_FRAME_BODY_NED,
        0b0000111111000111,
        0, 0, 0,
        velocity_x, velocity_y, velocity_z,
        0, 0, 0,
        0, 0
    )
    for _ in range(duration):
        vehicle.send_mavlink(msg)
        time.sleep(1)

def get_distance_metres(aLocation1, aLocation2):
    """Calculate distance between two GPS coordinates"""
    dlat = aLocation2.lat - aLocation1.lat
    dlon = aLocation2.lon - aLocation1.lon
    return math.sqrt((dlat*dlat) + (dlon*dlon)) * 1.113195e5

def goto_and_return(vehicle, target_location, cruise_speed=5):
    """Go to target location, wait, then return to starting point"""
    home_location = vehicle.location.global_relative_frame
    vehicle.groundspeed = cruise_speed
    
    print("[DRONA] Zbor către punctul țintă...")
    vehicle.simple_goto(target_location)
 
    while get_distance_metres(vehicle.location.global_relative_frame, target_location) > 2:
        print("[DRONA] Apropiere de punctul țintă...")
        time.sleep(1)

    print("[DRONA] Ajuns la destinație. Aștept 3 secunde...")
    time.sleep(3)

    print("[DRONA] Revenire la punctul inițial...")
    vehicle.simple_goto(home_location)
    while get_distance_metres(vehicle.location.global_relative_frame, home_location) > 2:
        print("[DRONA] Apropiere de poziția inițială...")
        time.sleep(1)

    print("[DRONA] Revenit la poziția inițială.")

def orbit_around_point(vehicle, center_location, radius=5, velocity=1.0, duration=20):
    """Orbit around a point using circular trajectory"""
    print("[DRONA] Încep orbitarea în jurul punctului...")
    set_roi(center_location, vehicle)

    angle = 0
    step_time = 1
    steps = int(duration / step_time)

    for _ in range(steps):
        vx = -velocity * math.sin(angle)
        vy = velocity * math.cos(angle)
        send_ned_velocity(vx, vy, 0, 1, vehicle)
        angle += (velocity / radius) * step_time
        time.sleep(0.1)

    print("[DRONA] Orbită completă sau întreruptă.")

# Initialize GPS provider
gps_provider = MockGPSProvider() if USE_SIMULATOR else DroneKitGPSProvider(bypass=False)

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

def detection_thread():
    global frame_buffer, yolo_output_frame, detected_flag, popup_sent, last_detection_time, frame_counter, event_location
    cam_x, cam_y = 320, 240
    PIXELS_PER_CM = 10
    object_present = False
    logging.info("Firul 2 (detectie) a pornit.")
    
    while True:
        if not streaming:
            time.sleep(0.1)
            continue
        frame_counter += 1
        if frame_counter % detection_frame_skip != 0:
            time.sleep(0.01)
            continue
            
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
            
        with output_lock:
            yolo_output_frame = cv2.imencode('.jpg', annotated)[1].tobytes()
        time.sleep(0.01)

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
        jpeg = cv2.imencode('.jpg', data["image"])[1].tobytes()
        with output_lock:
            output_frame = jpeg
        time.sleep(0.05)

# === Flask Routes ===
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

@app.route("/start_stream")
def start_stream():
    global streaming
    streaming = True
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
    def takeoff_task():
        #GUIDED,STABILIZE
        return gps_provider.arm_and_takeoff(2,"GUIDED")
    
    start_thread(takeoff_task, "TakeoffThread")
    return jsonify({"status": "takeoff initiated"})


@app.route("/land")
def land():
    def land_task():
        return gps_provider.land_drone()
    
    start_thread(land_task, "LandThread")
    return jsonify({"status": "landing initiated"})

@app.route("/return_to_event")
def return_to_event():
    global event_location
    logging.info("[FLASK] /return_to_event apelat")
    if not event_location:
        logging.warning("[FLASK] Nu există coordonate salvate pentru revenirea dronei.")
        return jsonify({"status": "no event location"})

    try:
        gps_provider.ensure_connection()
        logging.info(f"[FLASK] Se trimite drona înapoi la: {event_location}")
        gps_provider.vehicle.simple_goto(event_location)
        return jsonify({
            "status": "returning",
            "lat": event_location.lat,
            "lon": event_location.lon,
            "alt": event_location.alt
        })
    except Exception as e:
        logging.error(f"[FLASK] Eroare la trimiterea dronei către locație: {e}")
        return jsonify({"status": "error", "message": str(e)})

@app.route("/goto_and_return")
def goto_and_return_route():
    global event_location
    if not event_location:
        return jsonify({"status": "no event location"})

    try:
        gps_provider.ensure_connection()
        start_thread(lambda: goto_and_return(gps_provider.vehicle, event_location,4), "GotoReturnThread")
        return jsonify({"status": "going and returning"})
    except Exception as e:
        return jsonify({"status": "error", "message": str(e)})

@app.route("/orbit")
def orbit_route():
    global event_location
    try:
        gps_provider.ensure_connection()
    except:
        return jsonify({"status": "drone connection failed"})

    if not event_location:
        return jsonify({"status": "no event location"})

    try:
        start_thread(lambda: orbit_around_point(gps_provider.vehicle, event_location, radius=5, velocity=1, duration=30), "OrbitThread")
        return jsonify({"status": "orbiting"})
    except Exception as e:
        return jsonify({"status": "error", "message": str(e)})
        
@socketio.on('drone_command')
def handle_drone_command(data):
    action = data.get('action')
    print(f"[WS] Comandă primită: {action}")
    
    if action == 'takeoff':
        start_thread(lambda: gps_provider.arm_and_takeoff(2, "GUIDED"), "WS_Takeoff")
    elif action == 'land':
        start_thread(lambda: gps_provider.land_drone(), "WS_Land")
    elif action == 'goto_and_return':
        if event_location:
            start_thread(lambda: goto_and_return(gps_provider.vehicle, event_location, 4), "WS_GoRet")
    elif action == 'orbit':
        if event_location:
            start_thread(lambda: orbit_around_point(gps_provider.vehicle, event_location), "WS_Orbit")
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
        
def status_broadcast_loop():
    while True:
        try:
            if gps_provider.connected and gps_provider.vehicle:
                socketio.emit('drone_status', {
                    "connected": True,
                    "battery": {"level": gps_provider.vehicle.battery.level},
                    "armed": gps_provider.vehicle.armed,
                    "mode": gps_provider.vehicle.mode.name,
                    "location": {
                        "lat": gps_provider.vehicle.location.global_frame.lat,
                        "lon": gps_provider.vehicle.location.global_frame.lon
                    },
                    "event_location": {
                        "lat": event_location.lat if event_location else None,
                        "lon": event_location.lon if event_location else None
                    }
                })
        except Exception as e:
            print(f"[STATUS ERROR] {e}")
        time.sleep(1)

start_thread(status_broadcast_loop, "StatusBroadcast")

if __name__ == "__main__":
    start_thread(camera_thread, "CameraThread")
    start_thread(detection_thread, "DetectionThread")
    start_thread(stream_thread, "StreamThread")

    logging.info("Pornire server Flask + SocketIO")
    socketio.run(app, host="0.0.0.0", port=5000)

