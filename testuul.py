from picamera2 import Picamera2
import time

picam2 = Picamera2()
picam2.start()
time.sleep(2)
frame = picam2.capture_array()
print("Frame shape:", frame.shape)
picam2.close()
