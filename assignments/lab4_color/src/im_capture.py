#!/usr/bin/env python
'''
Simple image viewer and capturer:
 rosrun lab4_color im_capture.py        : subscribes to /raspicam_not/image/compressed 

Complete functions as indicated

'''
import os
import argparse
import rospy
import cv2
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge, CvBridgeError

class im_show:
    
    def __init__(self, outfolder='im_save'):
        self.bridge = CvBridge()
        self.outfolder = outfolder
        self.index = 0
        rospy.Subscriber('/raspicam_node/image/compressed', CompressedImage, self.display)
        rospy.loginfo('Image capture from raspberry topic.  Press "s" to save, "q" to quit.')

    def display(self, msg):
        ''' Uses OpenCV to display an image and waits 2ms
            If the user presses 's', then calls save() with the image
            If the user presses 'q', then quits 
        '''
        try:
            img = self.bridge.compressed_imgmsg_to_cv2(msg, 'bgr8')
        
        except CvBridgeError as e:
            print(e)

        cv2.imshow("Image", img)
        inp = cv2.waitKey(2)

        if inp == ord('s'):
            self.save(img)

        elif inp == ord('q'):
            rospy.signal_shutdown("Received 'q' message")

    def save(self, img):
        ''' Saves an image to file '''
        os.makedirs(self.outfolder,exist_ok=True)
        name = os.path.join( self.outfolder, 'img_' + "{:03d}.jpg".format(self.index) )
        rospy.loginfo('Writing: '+name)
        cv2.imwrite( name, img )
        self.index += 1

if __name__ == '__main__':

    parser = argparse.ArgumentParser(description='Image Capture from ros topic')
    parser.add_argument('--folder', default='im_save', type=str,    help='Folder to save images in')
    args, unknown = parser.parse_known_args()  # For roslaunch compatibility
    if unknown: print('Unknown args:',unknown)

    rospy.init_node('im_capture', anonymous=True)  # Start a ROS node
    ic = im_show( args.folder )
    try:
        rospy.spin()
    except KeyboardInterrupt:
        pass
    cv2.destroyAllWindows()

