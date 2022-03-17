import time
import gpiozero as GPIO
from gpiozero.pins.pigpio import PiGPIOFactory

factory = PiGPIOFactory()
servo_pin = 18
s = GPIO.AngularServo(servo_pin, initial_angle=0, min_angle=0, max_angle=180, min_pulse_width=0.0006, max_pulse_width=0.0024, pin_factory=factory)

try:
	while True:
		user_in = input("Enter input:")	
		user_in = int(user_in)
		print("Turning to: ", user_in)
		s.angle = user_in

except KeyboardInterrupt:
	print("Test over")

