#!/usr/bin/env python

import argparse
import cv2
import rospy
from dnn_detect import Dnn
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError


class Detect:
	def __init__(self, model_folder, threshold):
		self.bridge = CvBridge()
		self.dnn = Dnn(model_folder, "traffic.mp4", threshold)

		rospy.init_node('detect')
		rospy.Subscriber('cam_front/raw', Image, self.video)

	def video(self, msg):
		try:
			frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")

		except CvBridgeError:
			print(CvBridgeError)

		self.dnn.detect(frame)
		if cv2.waitKey(1) == ord('q'):
			rospy.signal_shutdown('quitting')


if __name__ == "__main__":
	parser = argparse.ArgumentParser(description='Image detection from dnn detects')
	parser.add_argument('folder', default=False, type=str, help='Folder data is stored in')
	parser.add_argument('--nms', default=0.4, type=float, help='NMS Threshold')
	args, unknown = parser.parse_known_args()

	if unknown: 
		print('Unknown args:',unknown)
		exit()

	det = Detect(args.folder, args.nms)

	try:
		rospy.spin()
	
	except KeyboardInterrupt:
		pass

	cv2.destroyAllWindows()