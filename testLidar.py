import time

bottom_rangefinder = 173

# Increase altitude
while True:
  current_altitude = vehicle.rangefinder[bottom_rangefinder].distance
  print("Altitude: %f"%current_altitude*1.0)
  if current_altitude >= 1:
    print("Reached target altitude")
    break
  time.sleep(0.2)