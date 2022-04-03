#!/usr/bin/env python
'''
    CircleDrive.py 

    This is an exmple ROS node for driving a Turtlebot in a circular arc and then stopping.

    Daniel Morris, Nov 2020
'''
import argparse
import rospy
from geometry_msgs.msg import Twist

class CircleDrive:
    def __init__(self):
        rospy.init_node('move')
        self.pub = rospy.Publisher('/cmd_vel', Twist, latch=True, queue_size=1)
            # latch: keeps published topic current until next published topic
	
    def start(self, linear, angular ): 
        move = Twist()                 # Initialized all-zero values
        move.linear.x = linear         # Linear velocity in the x axis
        move.angular.z = angular       # Angular velocity in the z axis
        self.pub.publish(move)  

    def stop(self): 
        move = Twist()                 # Initialized all-zero values
        self.pub.publish(move)  

    def drive(self, time=2.0, linear=2.0, angular=1.5):
        info = f'Driving for {time} sec with linear speed {linear} m/s and angular speed {angular} rad/s'
        rospy.loginfo(info)
        self.start( linear, angular )
        rospy.sleep( time )
        self.stop()
        rospy.sleep( 0.1 )     # Ensure publishing completes before quitting
        rospy.loginfo('Done driving')

if __name__== '__main__':
    parser = argparse.ArgumentParser(description='Circular Arc Arguments')
    parser.add_argument('--time',    default=10.0, type=float,    help='Time to drive')
    parser.add_argument('--linear',  default=0.2, type=float,    help='Linear speed')
    parser.add_argument('--angular', default=0.8, type=float,    help='Angular speed')
    args, unknown = parser.parse_known_args()  # For roslaunch compatibility
    if unknown: print('Unknown args:',unknown)

    av = CircleDrive()
    av.drive(args.time, args.linear, args.angular)
