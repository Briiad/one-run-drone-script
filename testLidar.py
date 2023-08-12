import time
from dronekit import connect, VehicleMode, LocationGlobalRelative, LocationGlobal

bottom_rangefinder = 173

vehicle = connect('localhost:14550', baud=57600, wait_ready=False)

# Increase altitude
while True:
  current_altitude = vehicle.rangefinder.distance
  print("Altitude: %f"%current_altitude)
  if current_altitude >= 1:
    print("Reached target altitude")
    break
  time.sleep(0.2)