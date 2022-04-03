#!/usr/bin/env python
'''
Detects multiple targets and publishes their centroids

 rosrun lab4_color multi_dot_detect.py imgname maskname 

A logistic regression classifier for targets is created from:
 imgname: name of saved image
 maskname: name of mask for saved image indicating which pixels are target pixels

'''
import argparse
import rospy
import cv2
from std_msgs.msg import Header
from sensor_msgs.msg import CompressedImage
from geometry_msgs.msg import Point
from lab4_color.msg import Point32Array
from cv_bridge import CvBridge, CvBridgeError
from logist_reg import LogisticReg, plotTargets
import message_filters

class PlotMultiDot:
    
	def __init__(self):
		dot_sub = message_filters.Subscriber('/dots', Point32Array)
		img_sub = message_filters.Subscriber('/raspicam_node/image/compressed', CompressedImage)
		self.ts = message_filters.TimeSynchronizer([dot_sub, img_sub], 2)
		self.ts.registerCallback(self.detect)  
		self.bridge = CvBridge()


	def detect(self, dot_msg, img_msg):
		img = self.bridge.compressed_imgmsg_to_cv2(img_msg,'bgr8')
		points = dot_msg.points
		for point in points:
			img = cv2.circle(img, ((int)(point.x), (int)(point.y)), 5, (0,0,255), -1)
        
		cv2.imshow("Image",img)
		cv2.waitKey(2)


if __name__ == '__main__':
	rospy.init_node('plot_multi_dot')
	PlotMultiDot()

	try:
		rospy.spin()

	except KeyboardInterrupt:
		pass

	cv2.destroyAllWindows()