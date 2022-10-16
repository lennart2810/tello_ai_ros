#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge


class CameraNode(object):
    def __init__(self):
            
        # init node and image publisher
        rospy.init_node('camera_node', anonymous=False)
        self.pub = rospy.Publisher('/camera_view', Image, queue_size=1)

        # cv2_bridge
        self.bridge = CvBridge()

        self.cap = cv2.VideoCapture(0)

        # run node
        while not rospy.is_shutdown():
                
            _, frame = self.cap.read()
            camera_view = self.bridge.cv2_to_imgmsg(frame, 'bgr8')
            self.pub.publish(camera_view)
            rospy.sleep(0.001)

        # release cap
        self.cap.release()


if __name__ == '__main__':
    try:
        CameraNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass