#!/usr/bin/env python

import rospy
import numpy as np
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, PointStamped
from tf.transformations import euler_from_quaternion
from pid import pid_controller
import matplotlib.pyplot as plt
import argparse

class follow():
  def __init__(self, speed):
    if speed > 0.5:
      speed = 0.5

    self.speed = speed
    self.target_yaw = 0
    self.use_rate = True
    self.pid = pid_controller(2, 0, 1)

    self.target_yaw_list = []
    self.meas_yaw_list = []
    self.cmd_yaw_rate_list = []
    self.meas_yaw_rate_list = []
    self.time_list = []

    rospy.init_node('pid_test')        
    self.pub_vel = rospy.Publisher('/cmd_vel', Twist, latch=True, queue_size=1)

    rospy.Subscriber('/odom', Odometry, self.callback_odom, queue_size=1)  # Important to have queue size of 1 to avoid delays
    rospy.Subscriber('/dot', PointStamped, self.callback_point, queue_size=1)

  def callback_point(self, msg: PointStamped):
    self.target_yaw = (140 - msg.point.x)/200
     

  def callback_odom(self, msg: Odometry):
    self.time_list.append(rospy.Time.now().to_sec())

    orientation = msg.pose.pose.orientation        
    angular = msg.twist.twist.angular
    _, _, current_yaw  = euler_from_quaternion([orientation.x, orientation.y,\
                                                orientation.z, orientation.w])
    self.meas_yaw_list.append(current_yaw)     # Measure yaw angle from IMU + odometry
    self.meas_yaw_rate_list.append(angular.z)  # Measure yaw rate from IMU
    target_yaw = current_yaw + self.target_yaw


    self.target_yaw_list.append(target_yaw)
    if self.use_rate:
        cmd_yaw_rate = self.pid.update_control_with_rate(target_yaw, current_yaw, self.meas_yaw_rate_list[-1])
    else:
        cmd_yaw_rate = self.pid.update_control(target_yaw, current_yaw)
    self.cmd_yaw_rate_list.append(cmd_yaw_rate)

    msg_twist = Twist()
    msg_twist.angular.z = cmd_yaw_rate
    msg_twist.linear.x = self.speed
    self.pub_vel.publish(msg_twist)
    self.plot()

  def plot(self):
    fig = plt.figure('PID',figsize=(8,4))
    fig.clf()
    plt.title(f'PID: ({self.pid.kp:.1f}, {self.pid.ki:.1f}, {self.pid.kd:.1f}), Rate: {self.use_rate:d}')
    plt.xlabel('Time (sec)')
    plt.ylabel('Heading (rad)')
    plt.grid(True, linestyle='--')
    plt.plot(np.array(self.time_list)-self.time_list[0], np.array(self.target_yaw_list),  'r-', label='Target Yaw')
    plt.plot(np.array(self.time_list)-self.time_list[0], np.array(self.meas_yaw_list),  'b-', label='Measured Yaw')
    plt.legend()
    plt.tight_layout()
    plt.gcf().canvas.flush_events()
    plt.show(block=False)
    plt.show(block=False)


if __name__=='__main__':
  parser = argparse.ArgumentParser(description='Move Turtlebot')
  parser.add_argument('--speed', default=0.5, help='Set speed of turtlebot', )
  
  args, unknown = parser.parse_known_args()  # For roslaunch compatibility
  if unknown: print('Unknown args:',unknown)

  follow(float(args.speed))

  rospy.spin()   
