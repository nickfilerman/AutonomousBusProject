#!/usr/bin/env python

import rospy
import numpy as np
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge, CvBridgeError
from logist_reg import LogisticReg
from geometry_msgs.msg import PointStamped

class LineDetect:
    
  def __init__(self, logr):
    rospy.init_node('line_detect')
    self.bridge = CvBridge()
    self.logr = logr
    rospy.Subscriber('camera/image/compressed', CompressedImage, self.detect)
    self.pub = rospy.Publisher('/dot', PointStamped)

  def detect(self, img_msg):
    try:
      img = self.bridge.compressed_imgmsg_to_cv2(img_msg,'bgr8')

    except CvBridgeError as e:
      print(e)

    img2 = img[-10:]

    prob_target = self.logr.apply(img2)
    centroid, _, target_mask = self.logr.find_largest_target(prob_target, threshold=0.5, minpix=20)

    if len(centroid) != 0:
      p = PointStamped(header=img_msg.header)

      p.point.x = centroid[0]
      p.point.y = centroid[1] - 10 + img.shape[0]
      p.point.z = 0

      self.pub.publish(p)

if __name__ == '__main__':
  lr = LogisticReg() 
  lr.cvec = np.array([[0.27621975, 0.1578597, -0.51132481]])
  lr.intercept = np.array([-19.83699926])

  dd = LineDetect(lr)

  try:
      rospy.spin()
  except KeyboardInterrupt:
      pass
