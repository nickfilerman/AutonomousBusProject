#!/usr/bin/env python

import rospy
import cv2
import numpy as np
from geometry_msgs.msg import Point
from sensor_msgs.msg import CameraInfo
from lab4_color.msg import Point32Array

class PixelRays:
	def __init__(self):
		rospy.Subscriber('/dots', Point32Array, self.detect)
		self.pub = rospy.Publisher('/rays', Point32Array)

	def detect(self, msg):
		msg.header.frame_id = 'camera_pose'
		pts = Point32Array(header = msg.header)

		for p in msg.points:
			src = np.ascontiguousarray([[p.x], [p.y]], np.float32)

			pt = cv2.undistortPoints(np.expand_dims(src, axis=1), K, D, R=None, P=None).reshape(2)
			pt = Point(pt[0], pt[1], 1)

			pts.points.append(pt)

		self.pub.publish(pts)


if __name__ == '__main__':
	rospy.init_node('pixel_rays')
	cam_msg = rospy.wait_for_message('/raspicam_node/camera_info', CameraInfo)
	D = np.array(cam_msg.D)
	K = np.array(cam_msg.K).reshape((3, 3))

	PixelRays()
	rospy.spin()