#!/usr/bin/env python3

import sys
import cv2
import rospy
from djitellopy import Tello
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from tello_ai_ros.srv import tello_service, tello_serviceResponse, tello_serviceRequest
#from tello_ai_ros.srv import *


class TelloNode(object):
    def __init__(self, mode):
            
        # init node, publisher, subscriber and service
        rospy.init_node('tello_node', anonymous=False)
        self.pub = rospy.Publisher('tello_view', Image, queue_size=10)
        rospy.Subscriber('cmd_vel', Twist, self.cmd_vel_callback, queue_size=1)
        self.prev_cmd_vel = Twist() # notwenig?? wrid nur in 76 verwendet!
        
        # tello button service
        rospy.Service('tello_button_service', tello_service, self.handle_tello_service)

        self.mode = mode
        if self.mode == 'normal' or self.mode == 'silent':
            self.tello = Tello()
            self.tello.connect()
            self.tello.streamon()
        elif self.mode == 'disabled':
            rospy.logwarn('The node started without connecting to the drone!')

        # cv2_bridge
        self.bridge = CvBridge()

        # run node
        while not rospy.is_shutdown():
            
            if self.mode == 'normal' or self.mode == 'silent':
                self.pub_tello_view()
            
            #self.print_debug_info()
            rospy.sleep(0.001) # notwendig??

        # release tello 
        if self.mode == 'normal' or self.mode == 'silent':
            self.tello.streamoff()
            self.tello.end()
        
        
    def pub_tello_view(self):

        # read frame from tello
        frame_read = self.tello.get_frame_read()
        frame = frame_read.frame

        # pub tello_view after converting with CvBridge
        tello_view = self.bridge.cv2_to_imgmsg(frame, 'bgr8')
        self.pub.publish(tello_view)


    def cmd_vel_callback(self, msg):

        if self.mode == 'normal':
            cmd_vel = Twist()
            cmd_vel.linear.x = int(msg.linear.x)
            cmd_vel.linear.y = int(msg.linear.y)
            cmd_vel.linear.z = int(msg.linear.z)
            cmd_vel.angular.z = int(msg.angular.z)
            
            # send_rc_control(self, left_right_velocity, forward_backward_velocity, up_down_velocity, yaw_velocity) 
            self.tello.send_rc_control(cmd_vel.linear.y, cmd_vel.linear.x, cmd_vel.linear.z, cmd_vel.angular.z)

        self.prev_cmd_vel = msg # notwenig??
        

    def handle_tello_service(self, req):
        
        if self.mode == 'normal':

            if req.request == 'button_cross':
                self.tello.takeoff()
            elif req.request == 'button_circle':
                self.tello.land()
            elif req.request == 'button_dpad_up':
                self.tello.flip_forward()
            elif req.request == 'button_dpad_down':
                self.tello.flip_back()
            elif req.request == 'button_share':
                print('take photo:')
                frame_read = self.tello.get_frame_read()
                frame = frame_read.frame
                cv2.imwrite('test.jpg', frame)

        elif self.mode == 'silent':

            if req.request == 'button_share':
                print('take photo:')
                frame_read = self.tello.get_frame_read()
                frame = frame_read.frame
                cv2.imwrite('test.jpg', frame)
            
        return tello_serviceResponse(req.request)

    
    def print_debug_info(self):
        text = "Battery: {}%".format(self.tello.get_battery())
        rospy.loginfo(text)
        #text = "State: {}%".format(self.tello.get_current_state())
        #rospy.loginfo(text)


if __name__ == '__main__':

    #filename = sys.argv[0]
    mode = sys.argv[1]

    try:
        TelloNode(mode)
        rospy.spin()
    except rospy.ROSInterruptException:
        pass