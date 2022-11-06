#!/usr/bin/env python3

import sys
import rospy
from ds4_driver.msg import Feedback
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from std_msgs.msg import String, Bool
from cv_bridge import CvBridge
import cv2
import numpy as np
from PoseDetectorClass import PoseDetector
from FaceDetectorClass import FaceDetector # was nodes.Face...
from tello_ai_ros.msg import object_position


class TelloControllerNode(object):
    def __init__(self, view, detector):
 
        # init node, publisher and subscriber
        rospy.init_node('tello_controller_node', anonymous=False)
        rospy.Subscriber('cmd_vel/rc', Twist, self.rc_cb, queue_size=1)
        rospy.Subscriber('object_position', object_position, self.object_position_cb, queue_size=1)

        #rospy.Subscriber('ai_control_flag', Bool, self.ai_control_flag_callback)
        self.ai_control_flag = True

        #self.pub_detection = rospy.Publisher('object_detection', Image, queue_size=1)
        #self.pub_position = rospy.Publisher('object_postion', object_position, queue_size=1)
        self.pub_vel = rospy.Publisher('cmd_vel', Twist, queue_size=1)

        # /set_feedback
        self.feedback = Feedback()
        self.feedback.set_led = True
        self.pub_feedback = rospy.Publisher('set_feedback', Feedback, queue_size=1)
        
        if detector == 'pose':
             pass
        elif detector == 'face':
             goal_params = rospy.get_param('~goal')
             self.goal = (goal_params['x'], goal_params['y'], goal_params['z'])

        # # cv2_bridge
        # self.bridge = CvBridge()

    def rc_cb(self, msg):
        if not self.ai_control_flag:
            self.pub_vel.publish(msg)



    def object_position_cb(self, msg):
        if self.ai_control_flag:
            print(msg)
        #   self.compute_cmd_vel_ai

    
    def ai_control_flag_callback(self, msg):
        if msg.data:
            self.ai_control_flag = True
        else:
            self.ai_control_flag = False


    # def frame_callback(self, msg):

    #     # read and reshape image data
    #     frame = np.frombuffer(msg.data, dtype=np.uint8)
    #     frame = frame.reshape(self.frame_shape)

    #     # get analysed image from FaceDetector
    #     found_object, object_frame, positions = self.detector.process_view(frame)

    #     # publish positions as object_position (own message x,y,l)

    #     if found_object:
    #         # display positions
    #         pos = object_position()
    #         pos.x = positions[0]
    #         pos.y = positions[1]
    #         pos.l = positions[2]
    #         print(pos)
    #         self.pub_position.publish(pos)

    #         cv2.putText(object_frame, f'x: {positions[0]} pix', (20, 40), cv2.FONT_HERSHEY_PLAIN,1, (255, 255, 255), 1)
    #         cv2.putText(object_frame, f'y: {positions[1]} pix', (20, 60), cv2.FONT_HERSHEY_PLAIN,1, (255, 255, 255), 1)
    #         cv2.putText(object_frame, f'z: {positions[2]} pix', (20, 80), cv2.FONT_HERSHEY_PLAIN,1, (255, 255, 255), 1)

    #         cmd_vel = self.compute_cmd_vel(positions)
    #     else:
    #         # rotate to find object
    #         cmd_vel = Twist()
    #         cmd_vel.angular.z = int(10)

    #     # pub the cmd_vel topic if enabling switch is active
    #     if self.ai_control_flag:
    #         self.pub_vel.publish(cmd_vel)
            
    #         # set controller led to green if object is detected, otherwise set led to red
    #         if found_object:
    #             self.feedback.led_r = 0; self.feedback.led_g = 1; self.feedback.led_b = 0
    #         else:
    #             self.feedback.led_r = 1; self.feedback.led_g = 0; self.feedback.led_b = 0
        
    #     # set led to blue for remote control
    #     else:
    #         self.feedback.led_r = 0; self.feedback.led_g = 0; self.feedback.led_b = 1

    #     self.pub_feedback.publish(self.feedback)
            
    #     # convert and pub analysed image 
    #     object_frame = self.bridge.cv2_to_imgmsg(object_frame, 'bgr8')
    #     self.pub_detection.publish(object_frame)


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
        
        p_x = 0.2
        p_y = 0.2
        p_z = 0.2 

        cmd_vel = Twist()
        cmd_vel.linear.x = int(-e_z * p_x)  # x.drohne --> vor und zurück in der ebene; hier wird abstand zum gesicht geregelt
        cmd_vel.linear.y = int(e_x * p_y) # y.drohne --> links rechts in der ebene; positionierung in der vertikalen achse 
        cmd_vel.linear.z = int(e_y * p_z) # z.drone --> hoch und runter entland der hochachse
        #cmd_vel.angular.z = int(msg.angular.z) ??? how to handle this ???

        return cmd_vel


if __name__ == '__main__':

    filename = sys.argv[0]
    view = sys.argv[1]
    detector = sys.argv[2]

    try:
        TelloControllerNode(view, detector)
        rospy.spin()
    except rospy.ROSInterruptException:
        pass