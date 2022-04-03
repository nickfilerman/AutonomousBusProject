#!/usr/bin/env python

import rospy
import cv2
from sensor_msgs.msg import CompressedImage
from geometry_msgs.msg import PointStamped
from cv_bridge import CvBridge, CvBridgeError
from logist_reg import plotTargets
import message_filters

class PlotDot:
  def __init__(self):
    rospy.init_node('PlotDot', anonymous=True)
    self.bridge = CvBridge()
    imsub = message_filters.Subscriber('camera/image/compressed', CompressedImage)
    dotsub = message_filters.Subscriber('dot', PointStamped)
    self.ts = message_filters.TimeSynchronizer([imsub, dotsub], queue_size=1)
    self.ts.registerCallback(self.plot_detections)        
    rospy.loginfo('Plotting detected targets')

  def plot_detections(self, img_msg, dot_msg):
    try:
      img = self.bridge.compressed_imgmsg_to_cv2(img_msg,'bgr8')
    except CvBridgeError as e:
      print(e)

    centroids = [[dot_msg.point.x,dot_msg.point.y]]  

    plotTargets(img, [], centroids )
    if (cv2.waitKey(2) & 0xFF) == ord('q'):
      rospy.signal_shutdown('Quitting')


if __name__ == '__main__':
  PlotDot()

  try:
    rospy.spin()
  except KeyboardInterrupt:
    pass

  cv2.destroyAllWindows()
