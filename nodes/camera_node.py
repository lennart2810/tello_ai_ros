#!/usr/bin/env python3

import cv2
import rospy
from cv_bridge import CvBridge
from sensor_msgs.msg import Image

class CameraNode(object):
    def __init__(self):
            
        # init node and image publisher
        rospy.init_node('camera_node', anonymous=False)
        self.pub = rospy.Publisher('/camera_view', Image, queue_size=1)

        # init cv2_bridge
        self.bridge = CvBridge()

        # init video capture (webcam)
        self.cap = cv2.VideoCapture(0)
        
        # run video stream publisher
        self.get_video_stream()


    def get_video_stream(self):
        
        while not rospy.is_shutdown():
            
            _, frame = self.cap.read()
            camera_view = cv2.flip(frame, 1) # for selfie view
            camera_view = self.bridge.cv2_to_imgmsg(camera_view, 'bgr8')
            self.pub.publish(camera_view)

        # release cap
        self.cap.release()



if __name__ == '__main__':
    try:
        CameraNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass