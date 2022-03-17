import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool


import RPi.GPIO as GPIO
from .pn532.pn532 import *
from .pn532.i2c import *
class CardReader(Node):

    def __init__(self):
        super().__init__('pub')
        self.publisher_ = self.create_publisher(Bool, 'NFC_Card', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0
        
        self.pn532 = PN532_I2C(debug=False, reset=20, req=16)
        ic, ver, rev, support = self.pn532.get_firmware_version()
        print('Found PN532 with firmware version: {0}.{1}'.format(ver, rev))
    
        # Configure PN532 to communicate with MiFare cards
        self.pn532.SAM_configuration()
        
        print('Waiting for RFID/NFC card...')
    
    def timer_callback(self):
        msg = Bool()
        msg.data = False
        
        #Code for NFC reader
        uid = self.pn532.read_passive_target(timeout=0.5)
        print('.', end="")
        #End of code for NFC reader 
        if uid is not None:
            msg.data = True
            print('Found an NFC Tag!')
        self.publisher_.publish(msg)
        self.get_logger().info('Presence of NFC tag: "%s"' % str(msg.data))
        self.i += 1


def main(args=None):
    rclpy.init(args=args)

    pub = CardReader()

    rclpy.spin(pub)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    pub.destroy_node()
    rclpy.shutdown()
    GPIO.cleanup()


if __name__ == '__main__':
    main()

