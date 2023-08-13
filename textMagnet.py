import Jetson.GPIO as GPIO

GPIO.setmode(GPIO.BOARD)
GPIO.setup(29, GPIO.OUT)

GPIO.output(29, GPIO.HIGH)