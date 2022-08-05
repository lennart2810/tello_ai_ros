#!/usr/bin/env python3

import sys
import rospy
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge
import cv2
import numpy as np
from FaceDetectorClass import FaceDetector

class DetectorNode(object):
    def __init__(self, view):
 
        # init node, publisher and subscriber
        rospy.init_node('detector_node', anonymous=False)
        rospy.Subscriber('tello_view', Image, self.frame_callback, queue_size=1)
        self.pub_detection = rospy.Publisher('object_detection', Image, queue_size=1)
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
        self.goal = (goal_params['x'], goal_params['y'], goal_params['z'])

        roi_params = rospy.get_param('~roi')
        roi = (roi_params['nose'], roi_params['eye_l'], roi_params['eye_r'])
        
        # create FaceDetector object
        self.detector = FaceDetector(self.frame_shape, roi)

        # cv2_bridge
        self.bridge = CvBridge()


    def frame_callback(self, msg):

        # read and reshape image data
        frame = np.frombuffer(msg.data, dtype=np.uint8)
        frame = frame.reshape(self.frame_shape)

        # get analysed image from FaceDetector
        found_object, object_frame, positions = self.detector.process_view(frame)

        if found_object:
            # calculate cmd_vel from target position and object position

            # hier infos der position auf das bild schreiben
            x_string = f'x: {positions[0]} pix'
            if positions[0] > 0:
                x_string += 'nach rechts: x --> y'
            elif positions[0] < 0:
                x_string += 'nach links: -x --> -y'
                    # display positions
            cv2.putText(object_frame, x_string, (20, 40), cv2.FONT_HERSHEY_PLAIN,1, (255, 255, 255), 1)
            cv2.putText(object_frame, f'y: {positions[1]} pix', (20, 60), cv2.FONT_HERSHEY_PLAIN,1, (255, 255, 255), 1)
            cv2.putText(object_frame, f'z: {positions[2]} pix', (20, 80), cv2.FONT_HERSHEY_PLAIN,1, (255, 255, 255), 1)

            cmd_vel = self.compute_cmd_vel(positions)
        else:
            # rotate to find object
            cmd_vel = Twist()
            cmd_vel.angular.z = int(10)

        # pub the cmd_vel topic 
        self.pub_vel.publish(cmd_vel)
            
        # convert and pub analysed image 
        object_frame = self.bridge.cv2_to_imgmsg(object_frame, 'bgr8')
        self.pub_detection.publish(object_frame)


    def compute_cmd_vel(self, positions):

        """
        image --> twist
        x - x
        y - z
        z - y ??

        horizontale Abweichung vom Objekt zum Ziel (e_x) resultiert in links - rechts Bewegung der Drohne (y-Achse)
        vertikaler Error (e_y) resultiert in auf - ab fliegen der Drohne (z-Achse)
        falsche Entfernung zu Ziel (e_z) resultiert in vor - zurück Bewegung der Drohne
        """

        e_x = positions[0] - self.goal[0]
        e_y = positions[1] - self.goal[1]
        e_z = positions[2] - self.goal[2]
        
        p_x = 0.1
        p_y = 0.1
        p_z = 0.1

        cmd_vel = Twist()
        cmd_vel.linear.x = int(e_z * p_x)
        cmd_vel.linear.y = int(-e_x * p_y)
        cmd_vel.linear.z = int(e_y * p_z)
        #cmd_vel.angular.z = int(msg.angular.z) ??? how to handle this ???

        return cmd_vel



if __name__ == '__main__':

    filename = sys.argv[0]
    view = sys.argv[1]

    try:
        DetectorNode(view)
        rospy.spin()
    except rospy.ROSInterruptException:
        pass