import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid

import numpy as np
import math
import cmath
import time

# NFC imports
from std_msgs.msg import Bool

# Servo Imports
from std_msgs.msg import Int8

rotatechange = 0.25

# code from https://automaticaddison.com/how-to-convert-a-quaternion-into-euler-angles-in-python/


def euler_from_quaternion(x, y, z, w):
    """
    Convert a quaternion into euler angles (roll, pitch, yaw)
    roll is rotation around x in radians (counterclockwise)
    pitch is rotation around y in radians (counterclockwise)
    yaw is rotation around z in radians (counterclockwise)
    """
    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x * x + y * y)
    roll_x = math.atan2(t0, t1)

    t2 = +2.0 * (w * y - z * x)
    t2 = +1.0 if t2 > +1.0 else t2
    t2 = -1.0 if t2 < -1.0 else t2
    pitch_y = math.asin(t2)

    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    yaw_z = math.atan2(t3, t4)

    return roll_x, pitch_y, yaw_z  # in radians


class AutoNav(Node):

    def __init__(self):
        super().__init__('auto_nav')
        
        self.nfc_msg = False
        self.nfc_found = False
        self.sleep = 10
        self.shot_balls = False

        # create publisher for moving TurtleBot
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)
        # self.get_logger().info('Created publisher')

        self.publisher_servo = self.create_publisher(Int8, 'servo', 10)

        # create subscription to track orientation
        self.odom_subscription = self.create_subscription(
            Odometry,
            'odom',
            self.odom_callback,
            10)
        # self.get_logger().info('Created subscriber')
        self.odom_subscription  # prevent unused variable warning
        # initialize variables
        self.roll = 0
        self.pitch = 0
        self.yaw = 0

        # create subscription to track occupancy
        self.occ_subscription = self.create_subscription(
            OccupancyGrid,
            'map',
            self.occ_callback,
            qos_profile_sensor_data)
        self.occ_subscription  # prevent unused variable warning
        self.occdata = np.array([])

        # create subscription to track lidar
        self.scan_subscription = self.create_subscription(
            LaserScan,
            'scan',
            self.scan_callback,
            qos_profile_sensor_data)
        self.scan_subscription  # prevent unused variable warning
        self.laser_range = np.array([])

        # create subscription to track NFC
        self.nfc_subscription = self.create_subscription(
            Bool,
            'NFC_Card',
            self.nfc_callback,
            1)

        # create subscription to track IRsensor
        self.ir_subscription = self.create_subscription(
            Int8,
            'IRsensor',
            self.ir_callback,
            1)


    def nfc_callback(self, msg):
        self.nfc_msg = msg.data
        if self.nfc_msg:
            self.get_logger().info('NFC detected!')

    def ir_callback(self, msg):
        if msg.data == 3:  # 2 Nothing detected, -1 turn left, 0 straight ahead, 1 turn right
            self.ir_msg = False
        elif msg.data == 2:
            self.ir_msg = "forward"
        elif msg.data == 0:
            self.ir_msg = "middle"
        elif msg.data == 1:
            self.ir_msg = "backward"
            
        if self.ir_msg:
            self.get_logger().info('Hot Target detected!')

    def odom_callback(self, msg):
        # self.get_logger().info('In odom_callback')
        orientation_quat = msg.pose.pose.orientation
        self.roll, self.pitch, self.yaw = euler_from_quaternion(orientation_quat.x, orientation_quat.y, orientation_quat.z, orientation_quat.w)

    def occ_callback(self, msg):
        # self.get_logger().info('In occ_callback')
        # create numpy array
        msgdata = np.array(msg.data)
        # compute histogram to identify percent of bins with -1
        # occ_counts = np.histogram(msgdata,occ_bins)
        # calculate total number of bins
        # total_bins = msg.info.width * msg.info.height
        # log the info
        # self.get_logger().info('Unmapped: %i Unoccupied: %i Occupied: %i Total: %i' % (occ_counts[0][0], occ_counts[0][1], occ_counts[0][2], total_bins))

        # make msgdata go from 0 instead of -1, reshape into 2D
        oc2 = msgdata + 1
        # reshape to 2D array using column order
        # self.occdata = np.uint8(oc2.reshape(msg.info.height,msg.info.width,order='F'))
        self.occdata = np.uint8(oc2.reshape(msg.info.height, msg.info.width))
        # print to file
        #np.savetxt(mapfile, self.occdata)

    def scan_callback(self, msg):
        #self.get_logger().info('In scan_callback')
        # create numpy array
        self.laser_range = np.array(msg.ranges)
        # print to file
        #np.savetxt(scanfile, self.laser_range)
        # replace 0's with nan
        self.laser_range[self.laser_range == 0] = np.nan

    def stop(self):
        self.get_logger().info('In stopbot')
        # publish to cmd_vel to move TurtleBot
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = 0.0
        # time.sleep(1)
        self.publisher_.publish(twist)

    def moveForward(self, speed):
        twist = Twist()
        twist.linear.x = speed
        twist.angular.z = 0.0  # initialize angular z to zero
        print('Is moving')
        self.publisher_.publish(twist)  # Publish the velocity

    def moveBackward(self, speed):
        twist = Twist()
        twist.linear.x = -speed
        # initialize angular z to zero  print('Is backing')
        twist.angular.z = 0.0
        self.publisher_.publish(twist)  # Publish the velocity

    # function to rotate the TurtleBot
    def rotatebot(self, rot_angle, modifier=1):
        self.get_logger().info('Angle to rotate is %f' % rot_angle)
        self.get_logger().info('In rotatebot')
        # create Twist object
        twist = Twist()
        
        # get current yaw angle
        current_yaw = self.yaw
        # log the info
        self.get_logger().info('Current: %f' % math.degrees(current_yaw))
        # we are going to use complex numbers to avoid problems when the angles go from
        # 360 to 0, or from -180sleep_time to 180
        c_yaw = complex(math.cos(current_yaw),math.sin(current_yaw))
        # calculate desired yaw
        target_yaw = current_yaw + math.radians(rot_angle)
        # convert to complex notation
        c_target_yaw = complex(math.cos(target_yaw),math.sin(target_yaw))
        self.get_logger().info('Desired: %f' % math.degrees(cmath.phase(c_target_yaw)))
        # divide the two complex numbers to get the change in direction
        c_change = c_target_yaw / c_yaw
        # get the sign of the imaginary component to figure out which way we have to turn
        c_change_dir = np.sign(c_change.imag)
        # set linear speed to zero so the TurtleBot rotates on the spot
        twist.linear.x = 0.0
        # set the direction to rotate
        twist.angular.z = c_change_dir * rotatechange * modifier
        # start rotation
        self.publisher_.publish(twist)

        # we will use the c_dir_diff variable to see if we can stop rotating
        c_dir_diff = c_change_dir
        # self.get_logger().info('c_change_dir: %f c_dir_diff: %f' % (c_change_dir, c_dir_diff))
        # if the rotation direction was 1.0, then we will want to stop when the c_dir_diff
        # becomes -1.0, and vice versa
        while(c_change_dir * c_dir_diff > 0):
            # allow the callback functions to run
            rclpy.spin_once(self)
            current_yaw = self.yaw
            # convert the current yaw to complex form
            c_yaw = complex(math.cos(current_yaw),math.sin(current_yaw))
            # self.get_logger().info('Current Yaw: %f' % math.degrees(current_yaw))
            # get difference in angle between current and target
            c_change = c_target_yaw / c_yaw
            # get the sign to see if we can stop
            c_dir_diff = np.sign(c_change.imag)
            # self.get_logger().info('c_change_dir: %f c_dir_diff: %f' % (c_change_dir, c_dir_diff))
            if not self.nfc_found and self.nfc_msg:
                self.nfc_found = True
                self.get_logger().info('Stopping robot from rotating for %d seconds' % self.sleep)
                self.stop()
                time.sleep(self.sleep)
                self.get_logger().info('Resuming rotatingbot now!')
                self.publisher_.publish(twist)
                continue
                        
        self.get_logger().info('End Yaw: %f' % math.degrees(current_yaw))
        # set the rotation speed to 0
        twist.angular.z = 0.0
        # stop the rotation
        self.publisher_.publish(twist)
        
    def check_nfc(self):
        if not self.nfc_found and self.nfc_msg:
            self.nfc_found = True
            self.get_logger().info('Stopping robot from moving for %d seconds' % self.sleep)
            self.stop()
            time.sleep(self.sleep)
            self.get_logger().info('Resuming moving now!')
            
            
    def actuate_servo(self):
        servo_msg = Int8()
        servo_msg.data = 1
        self.publisher_servo.publish(servo_msg)
        
    def adjust_to_target(self):
        speed = 0.005
        
        while self.ir_msg != 'middle':
            twist = Twist()
            twist.angular.z = 0
            if self.ir_msg == 'forward':
                twist.linear.x = speed
            elif self.ir_msg == 'backward':
                twist.linear.x = -speed
            self.publisher_.publish(twist)
            rclpy.spin_once(self)
        self.stop()
        
    def check_ir(self):
        #self.get_logger().info('Checking IR')
        if self.ir_msg and not self.shot_balls:
            self.stop()
            shoot_distance = 0.30 
            self.adjust_to_target()

            while np.nanmin(self.laser_range[range(265, 275, 1)]) < shoot_distance:
                twist = Twist()
                twist.angular.z = 0.0
                self.rotatebot(-15)
                twist.linear.x = -0.05
                self.publisher_.publish(twist)
                time.sleep(2)
                self.stop()
                self.rotatebot(15)
                twist.linear.x = 0.05
                self.publisher_.publish(twist)
                time.sleep(2)
                self.stop()
                
            self.adjust_to_target()
            self.rotatebot(-90)
            self.actuate_servo()
            time.sleep(8)
            self.rotatebot(90)


    def followWall(self):
        for _ in range(5):
            rclpy.spin_once(self)
            time.sleep(0.5)
            print(self.laser_range.size)
        print("Let's move the robot")
        # define the local speed
        
        speed = 0.15
        r_speed = 0.53
        threshhold = 0.03
        min_follow_dist = 0.2

        twist = Twist()

        # Execute the movement when the robot is active
        try:
            while rclpy.ok():
                # define the variable to determine the existance of the right wall
                no_right_wall = None
                # define the variable to determine the existance of the front wall
                no_front_wall = None
    
                # read the input distance between robot and obstacles in the
                # front, left, right, top left, and top right direction
                front = self.laser_range[1]
                front_left = self.laser_range[10]
                front_right = self.laser_range[350]
                top_left = self.laser_range[45]
                right = self.laser_range[270]
                top_right = self.laser_range[315]
                closest_right = np.nanmin(self.laser_range[range(260, 280, 1)])
                self.get_logger().info('Front = %f' % front)
                self.get_logger().info('Front_left = %f' % front_left)
                self.get_logger().info('Front_)right = %f' % front_right)
                self.get_logger().info('top_left = %f' % top_left)
                self.get_logger().info('right = %f' % right)
                self.get_logger().info('top_right = %f' % top_right)
                
                self.check_nfc()
                self.check_ir()
    
                if right< 0.35 or top_right <0.5:
                    no_right_wall = False  # False becuase right wall is detected
                    print('Right wall is detected')  # display message
    
                else:
                    no_right_wall = True  # True becuase right wall is not detected
                    print('Right wall is not detected')  # display message
    
                if front < 0.35 or front_left < 0.35 or front_right <0.35:
                    no_front_wall = False
                    print('Front wall is detected')
                else:
                    no_front_wall = True
                    print('Front wall is not detected')
    
                if no_right_wall:
                    if front < 0.3 or top_left < 0.42:
                        self.moveBackward(speed)
                        while front < 0.4 and rclpy.ok():
                            front = self.laser_range[1]
                            rclpy.spin_once(self)
                        self.stop()
    
                    if right > min_follow_dist:
                        twist.linear.x = 0.001
                    else:
                        twist.linear.x = 0.002
                    twist.angular.z = -0.5
                    self.publisher_.publish(twist)
    
                elif not no_right_wall and no_front_wall:
                    t_right = 1.414 * right
                    twist.linear.x = speed
                    twist.angular.z = 0.0
                    if (top_right < t_right - threshhold) or closest_right < min_follow_dist-0.05:
                        print(closest_right)
                        twist.angular.z = r_speed
                    elif top_right > t_right + threshhold or closest_right > min_follow_dist + 0.15:
                        twist.angular.z = -r_speed
                    self.publisher_.publish(twist)
    
                elif not no_right_wall and not no_front_wall:
                    if closest_right < min_follow_dist or front < 0.4 or top_right < 0.4:
                        twist.linear.x = 0.0
                    else:
                        twist.linear.x = 0.002
                    twist.angular.z = 0.5
                    self.publisher_.publish(twist)
    
                rclpy.spin_once(self)
                
        finally:
            self.stop()


def main(args=None):
    rclpy.init(args=args)

    auto_nav = AutoNav()
    try:
        auto_nav.followWall()
    finally:
        # Destroy the node explicitly
        # (optional - otherwise it will be done automatically
        # when the garbage collector destroys the node object)
        auto_nav.stop()
        auto_nav.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

