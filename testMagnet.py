import Jetson.GPIO as GPIO
import time

GPIO.setmode(GPIO.BOARD)
GPIO.setup(12, GPIO.OUT)

# set high for 10 seconds
GPIO.output(12, GPIO.HIGH)
time.sleep(10)

# set low
GPIO.output(12, GPIO.LOW)

GPIO.cleanup()