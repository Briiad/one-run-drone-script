import Jetson.GPIO as GPIO
import time

GPIO.setmode(GPIO.BOARD)
GPIO.setup(29, GPIO.OUT)

# set high for 10 seconds
GPIO.output(29, GPIO.HIGH)
time.sleep(10)

# set low
GPIO.output(29, GPIO.LOW)

GPIO.cleanup()