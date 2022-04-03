'''
    cmd_yaw.py

    Command a given yaw angle to the Turtlebot using a PID controller
    Applies a step function and plots both the command and the response
    Useful for selecting controller parameters: kp, ki, and kd

    Usage: first make sure the Turtlebot is running live or in Gazebo, then:

    python cmd_yaw.py <yaw_val> <kp> <ki> <kd> --use_rate

'''
import rospy
import numpy as np
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, PointStamped
from tf.transformations import euler_from_quaternion
from pid import pid_controller
import matplotlib.pyplot as plt
import argparse

class cmd_yaw():

    def __init__(self, target_yaw, kp, ki, kd, use_rate=False):

        self.target_yaw = target_yaw
        self.use_rate = use_rate
        self.pid = pid_controller(kp, ki, kd)

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

        dtime = self.time_list[-1] - self.time_list[0]

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
        msg_twist.linear.x = 0.1
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

    parser = argparse.ArgumentParser(description='Command Yaw')
    parser.add_argument('yaw', type=float, default=0.2, help='Yaw angle (rad)')
    parser.add_argument('kp', type=float, default=5, help='kp')
    parser.add_argument('ki', type=float, default=0., help='ki')
    parser.add_argument('kd', type=float, default=0., help='kd')
    parser.add_argument('--use_rate', action='store_true', default=False, help='Use IMU yaw rate')
    
    args, unknown = parser.parse_known_args()  # For roslaunch compatibility
    if unknown: print('Unknown args:',unknown)

    cmd_yaw(args.yaw, args.kp, args.ki, args.kd, args.use_rate)

    rospy.spin()   
