import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
import numpy as np

import time
import gpiozero as GPIO
from gpiozero.pins.pigpio import PiGPIOFactory

factory = PiGPIOFactory()
servo_pin = 18
s = GPIO.AngularServo(servo_pin, initial_angle=0, min_angle=0, max_angle=180, min_pulse_width=0.0006, max_pulse_width=0.0024, pin_factory=factory)

class Scanner(Node):

    def __init__(self):
        super().__init__('scanner')
        self.subscription = self.create_subscription(
            LaserScan,
            'scan',
            self.listener_callback,
            qos_profile_sensor_data)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        # create numpy array
        laser_range = np.array(msg.ranges)
        # replace 0's with nan
        laser_range[laser_range == 0] = np.nan
        
        for i in laser_range:
            if i<=1.1 and i>=1:
                    # log the info
                    self.get_logger().info('Laser Range: %f' % i)
                    actuate_servo(15)
                    actuate_servo(165)
                    break
	print('New laser scan came in!')


def actuate_servo(angle):
    s.angle = angle
    print('Turning to %i' % angle)
    time.sleep(1)
    return None

def main(args=None):
    rclpy.init(args=args)

    scanner = Scanner()

    rclpy.spin(scanner)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    scanner.destroy_node()
    rclpy.shutdown()  
    return None

if __name__ == '__main__':
    main()
