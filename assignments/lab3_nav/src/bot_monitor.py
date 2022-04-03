#!/usr/bin/env python

import rospy
import matplotlib.pyplot as plt
import math
from nav_msgs.msg import Odometry

class BotMonitor:
  def __init__(self, topic_name='/odom'):
    self.x = []
    self.y = []

    rospy.init_node("bot_monitor")
      
    rospy.Subscriber(topic_name, Odometry, self.callback)


  def callback(self, msg):
    x = round(msg.pose.pose.position.x, 3)
    y = round(msg.pose.pose.position.y, 3)

    if len(self.x) == 0 or len(self.y) == 0 or math.sqrt((x - self.x[-1])**2 + (y - self.y[-1])**2) > 0.02:
      self.x.append(x)
      self.y.append(y)

      plt.scatter(x, y)
      plt.gcf().canvas.flush_events()

    plt.grid(True, linestyle='--')
    plt.gca().set_aspect('equal', 'box')

    plt.show(block=False)


        

if __name__ == '__main__':
  monitor = BotMonitor()
  rospy.spin()

