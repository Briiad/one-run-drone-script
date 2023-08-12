# Import Important Libraries
import time
import math
import Jetson.GPIO as GPIO
import VL53L1X
import signal

# output pin 29
GPIO.setmode(GPIO.BOARD)
GPIO.setup(29, GPIO.OUT)

from JetsonCamera import Camera
from Focuser import Focuser

from dronekit import connect, VehicleMode, LocationGlobalRelative, LocationGlobal
from pymavlink import mavutil
import cv2

# Import Jetson Inference library
from jetson_inference import detectNet
from jetson_utils import cudaFromNumpy, videoSource

# Global variables
DEFAULT_TAKEOFF_THRUST = 0.7
SMOOTH_TAKEOFF_THRUST = 0.6
tof_run = True
bottom_rangefinder = 0

# Arm with RC transmitter
manualArm = True

# Start VL53L1X sensor
tof = VL53L1X.VL53L1X(i2c_bus=1, i2c_address=0x29)
tof.open()

tof.start_ranging(2)  # Start ranging, 1 = Short Range, 2 = Medium Range, 3 = Long Range

# Network
print("Loading network...")
net = detectNet(argv=['--model=../models/modelv1/ssd-mobilenet.onnx', '--labels=../models/modelv1/labels.txt', '--input-blob=input_0', '--output-cvg=scores', '--output-bbox=boxes'], threshold=0.5)

# Camera
print("Initializing camera...")
cam_1 = cv2.VideoCapture("gst-launch-1.0 nvarguscamerasrc sensor_id=0 ! video/x-raw(memory:NVMM), width=1280, height=720, format=NV12, framerate=30/1 ! nvvidconv flip-method=2 ! video/x-raw, format=BGRx ! videoconvert ! video/x-raw, format=BGR ! appsink")
cam_2 = cv2.VideoCapture("gst-launch-1.0 nvarguscamerasrc sensor_id=1 ! video/x-raw(memory:NVMM), width=1280, height=720, format=NV12, framerate=30/1 ! nvvidconv flip-method=1 ! video/x-raw, format=BGRx ! videoconvert ! video/x-raw, format=BGR ! appsink")

focuser_cam_1 = Focuser(7)
focuser_cam_1.set(Focuser.OPT_FOCUS, 150)
focuser_cam_2 = Focuser(8)
focuser_cam_2.set(Focuser.OPT_FOCUS, 150)

# Drone connection
print("Connecting to mavlink...")
mav_connect = mavutil.mavlink_connection('localhost:14551', baud=57600)
print("Connecting to drone...")
vehicle = connect('localhost:14550', baud=57600, wait_ready=False)

# dronekit check heartbeat
print("Waiting for heartbeat...")
vehicle.wait_ready('autopilot_version')
print("Heartbeat received!")

# FUNCTIONS FOR DISTANCE SENSOR
def exit_handler():
    global tof_run
    tof_run = False
    tof.stop_ranging()

# FUNCTIONS FOR NOGPS NAVIGATION

def send_attitude_target(roll_angle = 0.0, pitch_angle = 0.0, yaw_angle = None, yaw_rate = 0.0, use_yaw_rate = False, thrust = 0.5):
    if yaw_angle is None:
      yaw_angle = vehicle.attitude.yaw

    msg = vehicle.message_factory.set_attitude_target_encode(
      0,
      1,                                         #target system
      1,                                         #target component
      0b00000000 if use_yaw_rate else 0b00000100,#type mask: bit 1 is LSB
      to_quaternion(roll_angle, pitch_angle, yaw_angle), #q
      0,                                         #body roll rate in radian
      0,                                         #body pitch rate in radian
      math.radians(yaw_rate),                    #body yaw rate in radian/second
      thrust                                    #thrust
    )
    vehicle.send_mavlink(msg)

def set_attitude(roll_angle = 0.0, pitch_angle = 0.0, yaw_angle = None, yaw_rate = 0.0, use_yaw_rate = False, thrust = 0.5, duration = 0):
    send_attitude_target(roll_angle, pitch_angle, yaw_angle, yaw_rate, False, thrust)
    start = time.time()

    while time.time() - start < duration:
      send_attitude_target(roll_angle, pitch_angle, yaw_angle, yaw_rate, False, thrust)
      time.sleep(0.1)

    # Reset attitude, or it will persist for 1s more due to the timeout
    send_attitude_target(0, 0, 0, 0, True, thrust)

def to_quaternion(roll = 0.0, pitch = 0.0, yaw = 0.0):
    t0 = math.cos(math.radians(yaw * 0.5))
    t1 = math.sin(math.radians(yaw * 0.5))
    t2 = math.cos(math.radians(roll * 0.5))
    t3 = math.sin(math.radians(roll * 0.5))
    t4 = math.cos(math.radians(pitch * 0.5))
    t5 = math.sin(math.radians(pitch * 0.5))

    w = t0 * t2 * t4 + t1 * t3 * t5
    x = t0 * t3 * t4 - t1 * t2 * t5
    y = t0 * t2 * t5 + t1 * t3 * t4
    z = t1 * t2 * t4 - t0 * t3 * t5

    return [w, x, y, z]

# FUNTION FOR MISSION MOVEMENT

# Arm and takeoff
def arm_and_takeoff(targetHeight):   

    # Arming will be done via the RC transmitter
    if manualArm:
      print("Manual arming mode")
      while not vehicle.armed:
        print("Vehicle not armed, waiting...")
        time.sleep(1)
      print("Vehicle armed")

    # GUIDED_NOGPS mode for indoor flight
    vehicle.mode = VehicleMode("GUIDED_NOGPS")
    print("Vehicle mode set to GUIDED_NOGPS")
              
    # Set the takeoff thrust
    thrust = DEFAULT_TAKEOFF_THRUST

    # Take off to target height
    while True:
      # Use rangefinder to detect ground
      current_altitude = vehicle.rangefinder[bottom_rangefinder].distance
      print("Altitude: %f"%current_altitude*1.0)
      if current_altitude >= targetHeight*0.95:
        print("Reached target altitude")
        break
      elif current_altitude >= targetHeight*0.6:
        thrust = SMOOTH_TAKEOFF_THRUST
      set_attitude(thrust = thrust)
      time.sleep(0.2)

    return None

# Payload pickup
def pickup():
    # detect the paylod under the drone
    while True:
        res, frame = cam_2.read()

        if not res:
            print("Ignoring empty camera frame.")
            continue

        # Detect objects
        img = cudaFromNumpy(frame)
        detections = net.Detect(img)

        # square to draw in the middle of the frame ( 40 x 40 to make sure it's in the middle )
        cv2.rectangle(frame, (int(frame.shape[1] / 2) - 20, int(frame.shape[0] / 2) - 20), (int(frame.shape[1] / 2) + 20, int(frame.shape[0] / 2) + 20), (0, 255, 0), 2)

        for detection in detections:
            x = int(detection.Center[0])
            y = int(detection.Center[1])
            # draw circle on centroid
            cv2.circle(frame, (x, y), 5, (0, 0, 255), -1)
            # if centroid coordinates is in the middlle of the square, lower altitude
            if x >= int(frame.shape[1] / 2) - 20 and x <= int(frame.shape[1] / 2) + 20 and y >= int(frame.shape[0] / 2) - 20 and y <= int(frame.shape[0] / 2) + 20:
                time.sleep(0.5)
                # lower altitude until 0 m
                set_attitude(thrust = 4, duration=2)
                # engage electromagnet
                GPIO.output(29, GPIO.HIGH)
                time.sleep(0.5)
                break
            # if centroid outside the square, adjust position
            else:
                # if centroid coordinates is in the right side of the frame, roll right
                if x >= int(frame.shape[1] / 2) + 20:
                    set_attitude(roll_angle = -3, thrust = 0.5, duration=0.2)
                # if centroid coordinates is in the left side of the frame, roll left
                elif x <= int(frame.shape[1] / 2) - 20:
                    set_attitude(roll_angle = 3, thrust = 0.5, duration=0.2)
                # if centroid coordinates is in the bottom side of the frame, pitch backward
                elif y >= int(frame.shape[0] / 2) + 20:
                    set_attitude(pitch_angle = 3, thrust = 0.5, duration=0.2)
                # if centroid coordinates is in the top side of the frame, pitch forward
                elif y <= int(frame.shape[0] / 2) - 20:
                    set_attitude(pitch_angle = -3, thrust = 0.5, duration=0.2)
                    
        # Increase altitude
        while True:
            current_altitude = vehicle.rangefinder[bottom_rangefinder].distance
            print("Altitude: %f"%current_altitude*1.0)
            if current_altitude >= 1:
                print("Reached target altitude")
                break
            
            set_attitude(thrust = SMOOTH_TAKEOFF_THRUST)
            time.sleep(0.2)

        cam_2.release()
        break

# Search for drop zone
def search():
    while tof_run:
        distance_in_mm = tof.get_distance()
        distance_in_m = distance_in_mm / 1000
        print("Distance: %f m" % (distance_in_m))
        time.sleep(0.1)
        if distance_in_m <= 0.5:
            # pitch forward
            set_attitude(pitch_angle = -3, thrust = 0.5, duration=2)
            break
        else:
            # roll right
            set_attitude(roll_angle = -3, thrust = 0.5, duration=0.2)

# Drop zone detection
def drops():
    while True:
        res, frame = cam_1.read()

        if not res:
            print("Ignoring empty camera frame.")
            continue

        # Detect objects
        img = cudaFromNumpy(frame)
        detections = net.Detect(img)

        # line to draw in the middle of the frame ( +10 and -10 to make sure it's in the middle )
        cv2.line(frame, (int(frame.shape[1] / 2) - 10, 0), (int(frame.shape[1] / 2) - 10, frame.shape[0]), (0, 255, 0), 2)

        for detection in detections:
            # centroid coordinates
            x = int(detection.Center[0])
            y = int(detection.Center[1])
            # draw circle on centroid
            cv2.circle(frame, (x, y), 5, (0, 0, 255), -1)
            # draw bounding box
            cv2.rectangle(frame, (int(detection.Left), int(detection.Top)), (int(detection.Right), int(detection.Bottom)), (255, 0, 0), 2)
            # object class name and confidence
            cv2.putText(frame, "%s (%.1f%%)" % (detection.ClassID, detection.Confidence * 100), (int(detection.Left), int(detection.Top) - 5), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 2)

            # if centroid coordinates is in the middlle of the line, drop payload
            if x >= int(frame.shape[1] / 2) - 10 and x <= int(frame.shape[1] / 2) + 10:
                # Find the distance from the object using eucledian distance
                distance = math.sqrt((detection.Center[0] - int(frame.shape[1] / 2))**2 + (detection.Center[1] - int(frame.shape[0] / 2))**2)
                # if distance is less than 50, drop payload
                if distance <= 50:
                    # move forward a bit
                    set_attitude(pitch_angle = -3, thrust = 0.5, duration=0.2)
                    # disengage electromagnet
                    GPIO.output(29, GPIO.LOW)
                    # hover for 1 second
                    set_attitude(thrust = 0.5, duration=1)
                    break
            # if centroid outside the line, adjust position
            else:
                # if centroid coordinates is in the right side of the frame, roll right
                if x >= int(frame.shape[1] / 2) + 10:
                    set_attitude(roll_angle = -3, thrust = 0.5, duration=0.2)
                # if centroid coordinates is in the left side of the frame, roll left
                elif x <= int(frame.shape[1] / 2) - 10:
                    set_attitude(roll_angle = 3, thrust = 0.5, duration=0.2)
        
        # Display
        cv2.imshow("Frame", frame)
        # cv2.imshow("Frame CSI", frame_csi)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            cam_1.release()
            cv2.destroyAllWindows()
            break

# land
def land():
    # forward a bit
    set_attitude(pitch_angle = -3, thrust = 0.5, duration=1)
    print("Landing...")
    vehicle.mode = VehicleMode("LAND")
    while not vehicle.mode == "LAND":
        print("Waiting for LAND mode...")
        time.sleep(1)
    print("Vehicle in LAND mode")

# Main program
def main():
  arm_and_takeoff(1)
  # pickup()
  # search()
  # drops()
  # land()

# Run main program
if __name__ == "__main__":
  main()