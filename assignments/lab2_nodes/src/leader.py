#!/usr/bin/env python
''' leader.py

    This is a ROS node that defines room-A and robot-A. Robot-A is controlled 
    by the keyboard. Additionally the motion commands are published so other 
    nodes can follow.

    Your task is to complete the missing portions that are marked with
    # <...> 
'''
import rospy
import cv2
from std_msgs.msg import Int32
from robot_room import RobotRoom

class Leader():
    def __init__(self, topic_name='robot_move'):
        ''' Initialize Node -- we only want one leader to run 
            Define a publisher for a message of type Int32
            Initialize a RobotRoom
            Call listKeys() to tell user what to press
        '''
        rospy.init_node(topic_name)

        self.leader = rospy.Publisher(topic_name, Int32, queue_size=10)
        self.leader_x = rospy.Publisher("leader_x", Int32, queue_size=10)
        self.leader_y = rospy.Publisher("leader_y", Int32, queue_size=10)

        self.robot_room = RobotRoom(topic_name, color=(255,0,0))

        self.robot_room.listKeys()


    def run(self):
        ''' Create a loop that draws the robot, and based on the key pressed moves it accordingly 
            and publishes the key to the topic 
            Quit when the user presses a 'q'
        '''
        inp = 0

        while(inp != ord('q')):
            self.robot_room.move(inp)
            self.robot_room.draw()
            self.leader.publish(inp)
            self.leader_x.publish(self.robot_room.xy[1])
            self.leader_y.publish(self.robot_room.xy[0])

            inp = cv2.waitKey()

        self.leader.publish(inp)


        
if __name__ == '__main__':
    lead = Leader()
    lead.run()
