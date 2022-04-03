#!/usr/bin/env python

import rospy
import numpy as np
from scipy.spatial.transform import Rotation as Rot
from lab4_color.msg import Point32Array

class CalcRot:
	def __init__(self):
		rospy.Subscriber('/rays', Point32Array, self.detect)

	def detect(self, msg):
		x = np.array((msg.points[1].x, msg.points[1].y, msg.points[1].z))
		z = np.cross(np.array((msg.points[2].x, msg.points[2].y, msg.points[2].z)), np.array((msg.points[0].x, msg.points[0].y, msg.points[2].z)))

		mx = -1.0*x/np.linalg.norm(x)
		mz = 1.0*z/np.linalg.norm(z)
		my = np.cross(mz, mx)

		R = np.array((mx, my, mz))

		R = Rot.from_matrix(R)
		R = R.as_quat()

		arr = R.tolist()

		rospy.set_param('/cam_rot', arr)


if __name__ == '__main__':
	rospy.init_node('calc_rot')

	CalcRot()
	rospy.spin()