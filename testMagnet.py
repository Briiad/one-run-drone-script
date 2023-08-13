import Jetson.GPIO as GPIO
import time

GPIO.setmode(GPIO.BCM)
GPIO.setup(18, GPIO.OUT)

# set high for 10 seconds
GPIO.output(18, GPIO.HIGH)
time.sleep(10)

# set low
GPIO.output(18, GPIO.LOW)

GPIO.cleanup()