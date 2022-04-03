#!/usr/bin/env python

import argparse
import cv2
import rospy
from dnn_detect import Dnn
from std_msgs.msg import Bool
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError


class Detect:
	def __init__(self, model_folder, threshold):
		self.bridge = CvBridge()
		self.dnn = Dnn(model_folder, "traffic.mp4", threshold)
		self.force = False

		rospy.init_node('detect')
		rospy.Subscriber('dash_right/image_rect', Image, self.video)

		self.pub = rospy.Publisher('bus_stopping', Bool, queue_size=1)

	def video(self, msg):
		try:
			frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")

		except CvBridgeError:
			print(CvBridgeError)

		self.dnn.detect(frame, msg.header)

		if cv2.waitKey(1) == ord('q'):
			rospy.signal_shutdown('quitting')

		self.pub.publish(self.force or self.dnn.stop)

	def SetStop(self, should_stop):
		self.force = should_stop

	def GetStop(self):
		return self.force or self.dnn.stop


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