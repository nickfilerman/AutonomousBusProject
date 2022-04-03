#!/usr/bin/env python

import rospy
import numpy as np
from sensor_msgs import point_cloud2
from sensor_msgs.msg import PointCloud2, PointField, LaserScan

class Obstacles():
	def __init__(self):
		rospy.init_node('obstacles')
		rospy.Subscriber('/scan', LaserScan, self.lidar_callback)

		self.pub = rospy.Publisher('/obstacles', PointCloud2)

	def lidar_callback(self, msg: LaserScan):
		ranges = np.array(msg.ranges)
		angles = np.arange(len(ranges)) * msg.angle_increment + msg.angle_min
		my_list_x = []
		my_list_y = []
		raw_centroids = []

		for val in range(len(ranges) - 1):
			if ranges[val] < np.inf and ranges[val + 1] < np.inf:
				if abs(ranges[val] - ranges[val + 1]) < 0.25:
					my_list_x.append(ranges[val + 1] * np.cos(angles[val + 1]))
					my_list_y.append(ranges[val + 1] * np.sin(angles[val + 1]))
					
				else:
					x = np.array(my_list_x, np.float)
					y = np.array(my_list_y, np.float)

					raw_centroids.append([np.mean(x), np.mean(y),0.,0])

					my_list_x = []
					my_list_y = []

			else:
				x = np.array(my_list_x, np.float)
				y = np.array(my_list_y, np.float)

				raw_centroids.append([np.mean(x), np.mean(y),0.,0])

				my_list_x = []
				my_list_y = []

		fields = [PointField('x', 0, PointField.FLOAT32, 1),
					PointField('y', 4, PointField.FLOAT32, 1),
					PointField('z', 8, PointField.FLOAT32, 1),
					PointField('intensity', 12, PointField.UINT32, 1)]

		centroids = point_cloud2.create_cloud(msg.header, fields, raw_centroids)
		self.pub.publish(centroids)


if __name__ == '__main__':
	obstacles = Obstacles()
	rospy.spin()
