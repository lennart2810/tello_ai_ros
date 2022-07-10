#!/usr/bin/env python3

import sys
import rospy
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge
import numpy as np
from FaceDetectorClass import FaceDetector

class FaceDetectorNode(object):
    def __init__(self, view):
 
        # init node, publisher and subscriber
        rospy.init_node('face_detector_node', anonymous=False)
        rospy.Subscriber('/tello_view', Image, self.frame_callback, queue_size=1)
        self.pub_detection = rospy.Publisher('/face_detection', Image, queue_size=1)
        self.pub_vel = rospy.Publisher('cmd_vel', Twist, queue_size=1)

        # read params from tello.yaml
        if view == 'tello':
            frame_params = rospy.get_param('~tello_frame')
            self.frame_shape = (frame_params['w'], frame_params['h'], frame_params['d'])
            rospy.loginfo('Frame Shape: ' + view + str(self.frame_shape))
        elif view == 'camera':
            frame_params = rospy.get_param('~camera_frame')
            self.frame_shape = (frame_params['w'], frame_params['h'], frame_params['d'])
            rospy.loginfo('Frame Shape: ' + view + str(self.frame_shape))

        goal_params = rospy.get_param('~goal')
        goal = (goal_params['x'], goal_params['y'], goal_params['z'])

        roi_params = rospy.get_param('~roi')
        roi = (roi_params['nose'], roi_params['eye_l'], roi_params['eye_r'])
        
        # create FaceDetector object
        self.detector = FaceDetector(self.frame_shape, goal, roi)

        # cv2_bridge
        self.bridge = CvBridge()

    def frame_callback(self, msg):
        #frame = np.fromstring(msg.data, dtype=np.uint8)
        frame = np.frombuffer(msg.data, dtype=np.uint8)
        frame = frame.reshape(self.frame_shape)

        # get analysed image from FaceDetector
        found_face, face_frame, errors = self.detector.draw_infos_on_img(frame)

        #if found_face:

            # pub the cmd_vel topic 
        self.errors_to_cmd_vel(errors)
            
            # convert and pub analysed face image 
        face_frame = self.bridge.cv2_to_imgmsg(face_frame, 'bgr8')
        self.pub_detection.publish(face_frame)

        #else:
            #empty_frame = self.bridge.cv2_to_imgmsg(frame, 'bgr8')
            #self.pub_detection.publish(empty_frame)


        
    def errors_to_cmd_vel(self, errors):
        pass
        #cmd_vel_msg = Twist()
        #self.pub_vel.publish(cmd_vel_msg)

if __name__ == '__main__':

    filename = sys.argv[0]
    view = sys.argv[1]

    try:
        FaceDetectorNode(view)
        rospy.spin()
    except rospy.ROSInterruptException:
        pass