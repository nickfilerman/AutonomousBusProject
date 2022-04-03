#!/usr/bin/env python

import rospy
import math
import argparse
from transform_frames import TransformFrames
from geometry_msgs.msg import Twist, Point, PoseArray, Pose, TwistStamped, PointStamped
from nav_msgs.msg import Odometry

class SquarePursuit():
	def __init__(self, distance, speed):
		self.distance = distance
		self.speed = speed

		rospy.init_node('square_pursuit')
		self.pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
		self.twistpub = rospy.Publisher('cmd', TwistStamped, queue_size=1)
		self.pointpub = rospy.Publisher('point_ahead', PointStamped, queue_size=1)
		rospy.Subscriber('odom', Odometry, self.odom_callback, queue_size=1)

	def odom_callback(self, msg: Odometry):
		ls = self.get_intersections(msg.pose.pose.position.x, msg.pose.pose.position.y)
		pose_array = PoseArray(header=msg.header)
		tf = TransformFrames()

		for pair in ls:
			pt = Point(pair[0], pair[1], 0)

			ps = Pose()
			ps.orientation = msg.pose.pose.orientation
			ps.position = pt

			pose_array.poses.append(ps)

		next = None

		for po in tf.pose_transform(pose_array, 'base_footprint').poses:
			if not next or po.position.x > next[0]:
				next = (po.position.x, po.position.y)

		lin = msg.twist.twist.linear
		ang = msg.twist.twist.angular

		if next:
			k = (2*next[1])/(next[0]**2 + next[1]**2)
			
			ang.z = self.speed*k
			lin.x = ang.z/k

			new_twist = Twist()
			new_twist.angular = ang
			new_twist.linear = lin

			self.pub.publish(new_twist)

			twist_publish = TwistStamped(header=msg.header, twist=new_twist)
			self.twistpub.publish(twist_publish)
			
			pt2 = Point(next[0], next[1], 0)

			point_publish = PointStamped(header=msg.header, point=pt2)
			self.pointpub.publish(point_publish)

	def get_intersections(self, x, y):
		ls = []

		for i in range(360):
			xc = x + self.distance*math.cos(math.radians(i))
			yc = y + self.distance*math.sin(math.radians(i))

			if (math.isclose(yc, 0, abs_tol = 0.01) or math.isclose(yc, 1, abs_tol = 0.01)) and (xc >= 0 and xc <= 1):
				ls.append((round(xc, 4), round(yc, 4)))

			elif (math.isclose(xc, 0, abs_tol = 0.01) or math.isclose(xc, 1, abs_tol = 0.01)) and (yc >= 0 and yc <= 1):
				ls.append((round(xc, 4), round(yc, 4)))
			

		return ls	


if __name__ == '__main__':
	parser = argparse.ArgumentParser(description='Speed and Distance of Pure-Pursuit')
	parser.add_argument('--speed', default=0.2, type=float, help='Speed of Pure-Pursuit')
	parser.add_argument('--distance', default=0.4, type=float, help='Distance to Check of Pure-Pursuit')
	args = parser.parse_args()

	sp = SquarePursuit(args.distance, args.speed)

	try:
		rospy.spin()

	except KeyboardInterrupt:
		pass
