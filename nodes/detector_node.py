#!/usr/bin/env python3

import sys
import rospy
import numpy as np
import cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import Image

from tello_ai_ros.msg import object_position
from PoseDetectorClass import PoseDetector
from FaceDetectorClass import FaceDetector


class DetectorNode(object):
    def __init__(self, view, detector):
 
        # init node, publisher and subscriber
        rospy.init_node('detector_node', anonymous=False)
        rospy.Subscriber('tello_view', Image, self.frame_callback, queue_size=1)
        self.pub_detection = rospy.Publisher('object_detection', Image, queue_size=1)
        self.pub_position = rospy.Publisher('object_position', object_position, queue_size=1) 

        # read frame params from tello.yaml
        if view == 'tello':
            frame_params = rospy.get_param('~tello_frame')
        elif view == 'camera':
            frame_params = rospy.get_param('~camera_frame')
        elif view == 'gazebo':
            frame_params = rospy.get_param('~gazebo_frame')

        self.frame_shape = (frame_params['w'], frame_params['h'], frame_params['d'])
        rospy.loginfo('Frame Shape: ' + view + str(self.frame_shape))

        # create FaceDetector object
        if detector == 'pose':
            self.detector = PoseDetector(self.frame_shape)
        elif detector == 'face':
            roi_params = rospy.get_param('~roi') # roi mit in facedetector einbauen (hardcoden) ist bei pose ja quasi auch
            roi = (roi_params['nose'], roi_params['eye_l'], roi_params['eye_r'])
            self.detector = FaceDetector(self.frame_shape, roi)

        # cv2_bridge
        self.bridge = CvBridge()


    def frame_callback(self, msg):

        # read and reshape image data
        frame = np.frombuffer(msg.data, dtype=np.uint8)
        frame = frame.reshape(self.frame_shape)
        #rospy.logwarn(frame.shape)
        gray_image = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        #rospy.logwarn(gray_image.shape)
        #frame = frame.reshape(self.frame_shape)
        #rospy.logwarn(gray_image.shape)
        #gray_image = gray_image.reshape((480,640,1))

        # get analysed image from FaceDetector
        gray_image, positions = self.detector.process_view(gray_image) # danger bei face detector noch anpassen!!!

        pos = object_position()
        pos.detected = False

        if positions is not None:

            pos.detected = True
            pos.x = positions[0]
            pos.y = positions[1]
            pos.l = positions[2]
        
        self.pub_position.publish(pos)
            
        # convert and pub analysed image 
        #object_frame = self.bridge.cv2_to_imgmsg(object_frame, 'bgr8')
        gray_image = self.bridge.cv2_to_imgmsg(gray_image, 'mono8') 
        self.pub_detection.publish(gray_image)



if __name__ == '__main__':

    filename = sys.argv[0]
    view = sys.argv[1]
    detector = sys.argv[2]

    try:
        DetectorNode(view, detector)
        rospy.spin()
    except rospy.ROSInterruptException:
        pass