#!/usr/bin/env python
'''
    SquareDrive.py 

    This is an exmple ROS node for driving a Turtlebot in a square and then stopping.

    Nick Filerman, Sept 2021
'''
import argparse
import rospy
from geometry_msgs.msg import Twist

class SquareDrive:
    def __init__(self):
        rospy.init_node('move')
        self.pub = rospy.Publisher('/cmd_vel', Twist, latch=True, queue_size=1)
            # latch: keeps published topic current until next published topic
	
    def start(self, linear, angular): 
        move = Twist()                   # Initialized all-zero values
        move.linear.x = linear           # Linear velocity in the x axis
        move.angular.z = angular         # Angular velocity in the z axis
        self.pub.publish(move)

    def stop(self): 
        move = Twist()                   # Initialized all-zero values
        self.pub.publish(move)  

    def drive(self, time=1.0, linear=0.2):
        self.start(linear, 0)
        rospy.sleep(time*0.7)
        self.start(linear, 3.141593/2 + 0.05)
        rospy.sleep(1)

        self.start(linear, 0)
        rospy.sleep(time*0.4)
        self.start(linear, 3.141593/2 + 0.05)
        rospy.sleep(1)

        self.start(linear, 0)
        rospy.sleep(time*0.38)
        self.start(linear, 3.141593/2 + 0.05)
        rospy.sleep(1)

        self.start(linear, 0)
        rospy.sleep(time*0.7)

        rospy.sleep( 0.1 )     # Ensure publishing completes before quitting
        rospy.loginfo('Done driving')
        rospy.signal_shutdown('Done driving')

if __name__== '__main__':
    parser = argparse.ArgumentParser(description='Circular Arc Arguments')
    parser.add_argument('--time',    default=2.5, type=float,    help='Time to drive')
    parser.add_argument('--linear',  default=0.5, type=float,    help='Linear speed')
    args, unknown = parser.parse_known_args()  # For roslaunch compatibility
    if unknown: print('Unknown args:',unknown)

    av = SquareDrive()
    av.drive(args.time, args.linear)
