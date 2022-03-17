from gpiozero import LED
from time import sleep
from gpiozero.pins.pigpio import PiGPIOFactory

led = LED(21)

try:
    while True:
        user_in = input("Enter input(y/n):")
        if user_in == 'y':
            for _ in range(10):
                led.on()
                led.off()

                
        if user_in == 'n':
            print("Test over")
            break
        
except KeyboardInterrupt:
    print("Test over")
