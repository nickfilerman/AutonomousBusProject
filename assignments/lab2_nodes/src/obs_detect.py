#!/usr/bin/env python
''' obs_detect.py

    The start of assignment 1, Lab 2
    This is the outline of a ROS node that should subscribe to a PointCloud2 topic,
    filter it, and publish a filtered PointCloud2 topic

    Your task is to complete the missing portions that are marked with
    # <...> 

    Copyright: Daniel Morris, 2020
'''
import rospy
from sensor_msgs.msg import PointCloud2, PointField
from sensor_msgs import point_cloud2

class Obstacles():
    def __init__(self):
        # Initialize the node
        rospy.init_node("lab2_nodes")

        # Create a publisher for topic 'points_obs' that is of type: PointCloud2
        self.pub = rospy.Publisher("points_obs", PointCloud2, queue_size=10)
        
        # Define a subscriber for 'points_raw' that is type PointCloud2 and calls self.lidar_callback
        rospy.Subscriber("points_raw", PointCloud2, self.lidar_callback)

        rospy.loginfo("Node subscribes to: 'points_raw' and publishes to: 'points_obs'")

    def lidar_callback(self, msg):
        # Read the message into a list of points:
        pts = point_cloud2.read_points(msg, field_names=['x','y','z','intensity'])

        # Create a new list called pts_new which will contain the obstacle points
        pts_new = []
        for pt in pts:
            if pt[0] < 20 and 1.6 - pt[2] > 0.25 and 1.6 - pt[2] < 2.0:
                pts_new.append(pt)

        # Create a PointCloud2 message
        fields = [PointField('x', 0, PointField.FLOAT32,1),
                  PointField('y', 4, PointField.FLOAT32,1),
                  PointField('z', 8, PointField.FLOAT32,1),
                  PointField('intensity', 12, PointField.FLOAT32,1)]
        msg_new = point_cloud2.create_cloud(msg.header, fields, pts_new)
        # Note: This uses the same header as the input message to pass on the time it was taken

        # Publish the message        
        self.pub.publish(msg_new)

        
if __name__ == '__main__':
    Obstacles()

    rospy.spin() # Keeps python from exiting and handles callbacks
