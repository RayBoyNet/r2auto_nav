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

#NFC imports
from std_msgs.msg import Bool

#Servo Imports
from std_msgs.msg import Int8

# constants
rotatechange = 0.10
speedchange = 0.18
occ_bins = [-1, 0, 100, 101]
stop_distance = 0.25
front_angle = 30
front_angles = range(-front_angle,front_angle+1,1)
pick_angle = 90
pick_angles = range(-pick_angle,pick_angle+1,1)
scanfile = 'lidar.txt'
mapfile = 'map.txt'

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

    return roll_x, pitch_y, yaw_z # in radians

class AutoNav(Node):

    def __init__(self):
        super().__init__('auto_nav')
        
        # create publisher for moving TurtleBot
        self.publisher_ = self.create_publisher(Twist,'cmd_vel',10)
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
        
        self.nfc = False
        self.sleep = 5
        self.wall = None
        self.nfc_found = False

    def nfc_callback(self, msg):
        self.nfc = msg.data
        if self.nfc:
            self.get_logger().info('NFC detected!')
            
    
    def odom_callback(self, msg):
        # self.get_logger().info('In odom_callback')
        orientation_quat =  msg.pose.pose.orientation
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
        self.occdata = np.uint8(oc2.reshape(msg.info.height,msg.info.width))
        # print to file
        np.savetxt(mapfile, self.occdata)


    def scan_callback(self, msg):
        #self.get_logger().info('In scan_callback')
        # create numpy array
        self.laser_range = np.array(msg.ranges)
        # print to file
        #np.savetxt(scanfile, self.laser_range)
        # replace 0's with nan
        self.laser_range[self.laser_range==0] = np.nan


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
            if not self.nfc_found and self.nfc:
                self.nfc_found = True
                self.get_logger().info('Stopping robot from rotating for %d seconds' % self.sleep)
                self.stopbot()
                time.sleep(self.sleep)
                self.get_logger().info('Resuming rotatingbot now!')
                self.publisher_.publish(twist)
                continue
                        

        self.get_logger().info('End Yaw: %f' % math.degrees(current_yaw))
        # set the rotation speed to 0
        twist.angular.z = 0.0
        # stop the rotation
        self.publisher_.publish(twist)


    def pick_direction(self):
        self.get_logger().info('In pick_direction')
        if self.laser_range.size != 0:
            # use nanargmax as there are nan's in laser_range added to replace 0's
            lri = (self.laser_range[pick_angles]).nonzero()
            lr2i = np.nanargmax(list[lri])
            self.get_logger().info('Picked direction: %d %f m' % (lr2i, self.laser_range[lr2i]))
        else:
            lr2i = 0
            self.get_logger().info('No data!')

        # rotate to that direction
        self.rotatebot(float(lr2i))
        self.start_move()
        
        
    def start_move(self, modifier=1):
        # start moving
        self.get_logger().info('In Start Mover')
        twist = Twist()
        twist.linear.x = speedchange*modifier
        twist.angular.z = 0.0
        # not sure if this is really necessary, but things seem to work more
        # reliably with this
        time.sleep(1)
        self.publisher_.publish(twist)


    def stopbot(self):
        self.get_logger().info('In stopbot')
        # publish to cmd_vel to move TurtleBot
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = 0.0
        # time.sleep(1)
        self.publisher_.publish(twist)
                        
                    
    def mover(self):
        self.get_logger().info('In Mover')
        try:
            # initialize variable to write elapsed time to file
            # contourCheck = 1

            # find direction with the largest distance from the Lidar,
            # rotate to that direction, and start moving

            while rclpy.ok():
                if self.laser_range.size != 0:
                    # check distances in front of TurtleBot and find values less
                    # than stop_distance
                    lri = (self.laser_range[front_angles]<float(stop_distance)).nonzero()
                    # self.get_logger().info('Distances: %s' % str(lri))

                    # if the list is not empty    
                    if(len(lri[0])>0):
                        # stop moving
                        self.stopbot()
                        if not self.follow:
                            self.check_wall(30)
                            self.bouncebot()
                        elif self.follow:
                            return None
                    
                    #self.get_logger().info('NFC Data %s' % str(self.nfc))
                    if not self.nfc_found and self.nfc:
                        self.nfc_found = True
                        self.get_logger().info('Stopping robot from moving for %d seconds' % self.sleep)
                        self.stopbot()
                        time.sleep(self.sleep)
                        self.get_logger().info('Resuming mover now!')
                        self.start_move()
                        continue
                    
                # allow the callback functions to run
                rclpy.spin_once(self)

        except Exception as e:
            print(e)
        
        # Ctrl-c detected
        finally:
            # stop moving
            self.stopbot()

    def bouncebot(self):
        self.get_logger().info('In bouncebot')
        hyp = float(self.laser_range[0])
        min_lri = np.nanargmin(self.laser_range)
        opp = self.laser_range[min_lri]
        print(hyp, opp)
        if False: #type(hyp)!=float or opp > 1:
            self.pick_direction()
        else:
            angle = math.degrees(math.asin(opp/hyp))
            if self.wall == "left":
                # print("Angle to rotate is:", -2*angle)
                print("left")
                self.rotatebot(-2*angle)
                self.start_move()
                self.mover()
            elif self.wall =="right":
                # print("Angle to rotate is:", 2*angle)
                print("Right")
                self.rotatebot(2*angle)
                self.start_move()
                self.mover()
            else:
                self.pick_direction()
                self.mover()
                
    
    def first_rotate(self):
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = 1*rotatechange
        self.publisher_.publish(twist)
        print("here")
        time.sleep(2.5)
        twist.angular.z = 0.0
        self.publisher_.publish(twist)
        print("here")
        
        
    def check_wall(self, dist):
        self.get_logger().info('In checkwall')
        #self.get_logger().info('Laser size: %d' % self.laser_range.size)
        
        if self.laser_range.size != 0:
            lri = (self.laser_range[pick_angles]<float(dist)).nonzero()
        print(lri)
        
        if(len(lri[0])>0):
            l_count = 0
            r_count = 0
            for i in lri[0]:
                if i < pick_angle:
                    r_count += 1
                elif i> pick_angle:
                    l_count += 1
            if r_count > l_count:
                self.wall = "right"
            elif l_count > r_count:
                self.wall = "left" 
        self.get_logger().info('Wall detected on the: %s' % str(self.wall))
            
            
    def adjust_wall(self):
        self.get_logger().info('In adjust wall')
        twist = Twist()
        twist.linear.x = 0.0
        diff_checker = 0.003
        speed_change = 0.3
        
        if self.wall =="left":
            l1 = 60
            l2 = 120
            speed_change = -speed_change
            
        elif self.wall =="right":
            l1 = 300
            l2 = 240
         
        a1 = self.laser_range[l1]
        a2 = self.laser_range[l2]
        diff = a1 - a2
        self.get_logger().info('Adjusting to %s wall, diff = %f, angle 60 = %f, angle 120 = %f' % (self.wall, diff, a1, a2))
        
        if diff > diff_checker:
            #twist.angular.z = -speed_change*rotatechange #(-)Rotate Clockwise
            #self.publisher_.publish(twist)
            self.get_logger().info('Adjusting Robot to wall')
            while diff > diff_checker:
                self.rotatebot(-0.5, 0.3)
                time.sleep(0.2)
                rclpy.spin_once(self)
                a1= self.laser_range[l1]
                a2= self.laser_range[l2]
                diff = a1 - a2
                self.get_logger().info('diff = %f, angle 60 = %f, angle 120 = %f' % (diff, a1, a2))
            twist.angular.z = 0.0
            self.publisher_.publish(twist)
            
        elif diff < -diff_checker:
            #twist.angular.z = speed_change*rotatechange
            #self.publisher_.publish(twist)
            self.get_logger().info('Adjusting Robot to wall')
            while diff < -diff_checker:
                self.rotatebot(0.5, 0.3)
                time.sleep(0.2)
                rclpy.spin_once(self)
                a1= self.laser_range[l1]
                a2= self.laser_range[l2]
                diff = a1 - a2
                self.get_logger().info('diff = %f, angle 60 = %f, angle 120 = %f' % (diff, a1, a2))
                time.sleep(0.2)
            twist.angular.z = 0.0
            self.publisher_.publish(twist)
# --------------------------------------------        
#         if self.wall == "left":
#             angle_45 = self.laser_range[60]
#             angle_135 = self.laser_range[120]
#             diff = angle_45 - angle_135
#             self.get_logger().info('Adjusting to left wall, diff = %f, angle 45 = %f, angle 135 = %f' % (diff, angle_45, angle_135))
            
#             if diff > diff_checker:
#                 twist.angular.z = -speed_change*rotatechange #Rotate Clockwise
#                 self.publisher_.publish(twist)
#                 self.get_logger().info('Adjusting Robot to wall')
#                 while diff > diff_checker:
#                     rclpy.spin_once(self)
#                     angle_45 = self.laser_range[60]
#                     angle_135 = self.laser_range[120]
#                     diff = angle_45 - angle_135
#                     self.get_logger().info('diff = %f, angle 45 = %f, angle 135 = %f' % (diff, angle_45, angle_135))
#                 twist.angular.z = 0.0
#                 self.publisher_.publish(twist)
#             elif diff < -diff_checker:
#                 twist.angular.z = speed_change*rotatechange
#                 self.publisher_.publish(twist)
#                 self.get_logger().info('Adjusting Robot to wall')
#                 while diff < -diff_checker:
#                     angle_45 = self.laser_range[60]
#                     angle_135 = self.laser_range[120]
#                     rclpy.spin_once(self)
#                     diff = angle_45 - angle_135
#                     self.get_logger().info('diff = %f, angle 45 = %f, angle 135 = %f' % (diff, angle_45, angle_135))
#                 twist.angular.z = 0.0
#                 self.publisher_.publish(twist)

#         elif self.wall == "right":
#             angle_45 = self.laser_range[300]
#             angle_135 = self.laser_range[240]
            
#             diff = angle_45 - angle_135
#             self.get_logger().info('Adjusting to wall, diff = %f, angle 45 = %f, angle 135 = %f' % (diff, angle_45, angle_135))
            
#             if diff > diff_checker:
#                 twist.angular.z = speed_change*rotatechange
#                 self.publisher_.publish(twist)
#                 self.get_logger().info('Adjusting Robot to wall')
#                 while diff > diff_checker:
#                     rclpy.spin_once(self)
#                     angle_45 = self.laser_range[300]
#                     angle_135 = self.laser_range[240]
#                     diff = angle_45 - angle_135
#                     self.get_logger().info('diff = %f, angle 45 = %f, angle 135 = %f' % (diff, angle_45, angle_135))
#                 twist.angular.z = 0.0
#                 self.publisher_.publish(twist)
#             elif diff < -diff_checker:
#                 twist.angular.z = -speed_change*rotatechange
#                 self.publisher_.publish(twist)
#                 self.get_logger().info('Adjusting Robot to wall')
#                 while diff < -diff_checker:
#                     rclpy.spin_once(self)
#                     angle_45 = self.laser_range[300]
#                     angle_135 = self.laser_range[240]
#                     diff = angle_45 - angle_135
#                     self.get_logger().info('diff = %f, angle 45 = %f, angle 135 = %f' % (diff, angle_45, angle_135))
#                 twist.angular.z = 0.0
#                 self.publisher_.publish(twist)
                
    def follow_wall(self):     
        for _ in range(5):
            rclpy.spin_once(self)
            time.sleep(0.5)
            print(self.laser_range)
            print(self.nfc)
        
        self.rotatebot(10)
        self.get_logger().info('In follow_wall')
        self.follow = True 
        rotate = -1
        num_of_turn = 0
        self.check_wall(1.0)
        if self.wall == "right":
            rotate = 1
    
        while num_of_turn <= 3:
            self.adjust_wall()
            self.start_move(0.7)
            self.mover()
            self.rotatebot(rotate*90)
            num_of_turn += 1
            
            servo = Int8()
            servo.data = 0
            self.publisher_servo.publish(servo)
        
        self.follow = False
        self.start_move()
        self.mover()
            
        
        
def main(args=None):
    rclpy.init(args=args)
    
    auto_nav = AutoNav()

    auto_nav.follow_wall()

    # create matplotlib figure
    # plt.ion()
    # plt.show()

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    auto_nav.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
