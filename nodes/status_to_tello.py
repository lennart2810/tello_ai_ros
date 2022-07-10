#!/usr/bin/env python3

# -- ds4_to_tello_twist.py --
# Version vom 30.01.2022 by LF
# ClassDefinition StatusToTelloTwist
# ----------------------------


import sys
import rospy
#import subprocess
from geometry_msgs.msg import Twist
from ds4_driver.msg import Status, Feedback
from tello_ai_ros.srv import tello_service, tello_serviceRequest, tello_serviceResponse


class StatusToTello(object):
    def __init__(self):

        rospy.init_node('status_to_tello')
        
        # load params from status_to_tello.yaml file
        self.inputs = rospy.get_param('~inputs')
        self.scales = rospy.get_param('~scales')

        # /status
        self.attrs = []
        self.prev_status = Status()
        for attr in Status.__slots__:
            # benötigten Controller-Funktionen aus Status extrahieren (nochmal genauer aussortieren, um durchlaufzeit im callback zu reduzieren)
            # vllt über yaml file, in dem nur die verwendeten button und axis gelistet sind
            if attr.startswith('axis_') or attr.startswith('button_'):
                self.attrs.append(attr)
        rospy.Subscriber('status', Status, self.cb_status, queue_size=1)

        self.button_attrs = []
        self.axis_attrs = []
        for attr in Status.__slots__:
            if attr.startswith('axis_'):
                self.axis_attrs.append(attr)
            elif attr.startswith('button_'):
                self.button_attrs.append(attr)

        rospy.logwarn(self.button_attrs)
       
        # test = tello_serviceRequest.__slots__
        # print(test)
        # print(type(test))
        # print(len(test))
        # self.tello_service_request_template = [None] * len(test)

        # for i in range(0,len(self.tello_service_request_template)):
        #     self.tello_service_request_template[i] = False
        
        # print(self.tello_service_request_template)

        # /cmd_vel
        self.vel_msg = Twist
        self.pub_vel_flag = True
        self.pub_vel = rospy.Publisher('cmd_vel', self.vel_msg, queue_size=1)

        # /set_feedback
        self.feedback = Feedback()
        self.feedback.set_led = True
        self.feedback.set_led_flash = True
        self.pub_feedback = rospy.Publisher('set_feedback', Feedback, queue_size=1)


    def cb_status(self, msg):

        # Buttons
        self.cb_button(msg)

        # Axis
        self.cb_axis(msg)

        # /set_feedback <-- Batterieanzeige (Controller)
        if msg.battery_percentage <= 0.375:
            self.feedback.led_flash_on = 0.3
            self.feedback.led_flash_off = 0.3
        elif msg.battery_percentage <= 0.25:
            self.feedback.led_flash_on = 0.1
            self.feedback.led_flash_off = 0.1
        else:
            self.feedback.led_flash_off = 0

    def cb_button(self, status):
        # detect change in button status
        for attr in self.button_attrs:
            if getattr(status, attr) is not getattr(self.prev_status, attr): # flag in button status detected
                if getattr(status, attr) == True: # positive flag in one button status detected
                    
                    # prepare service call
                    rospy.wait_for_service('tello_button_service')
                    request = rospy.ServiceProxy('tello_button_service', tello_service)
                    req_msg = tello_serviceRequest(False, False, False, False)
                    response = ''

                    try:
                        if attr == 'button_dpad_up':
                            req_msg.up = True
                            response = request(req_msg)
                        elif attr == 'button_dpad_down':
                            req_msg.down = True
                            response = request(req_msg)
                        elif attr == 'button_dpad_left':
                            req_msg.left = True
                            response = request(req_msg)
                        elif attr == 'button_dpad_right':
                            req_msg.right = True
                            response = request(req_msg)

                        # buttons without service call
                        elif attr == 'button_trackpad':
                            rospy.loginfo("Battery: %s %%", ( status.battery_percentage * 100))
                            rospy.loginfo("USB: %s",  status.plug_usb)
                        
                        # rospy.loginfo('service response' + str(response))
                        # break

                    except rospy.ServiceException as e:
                        rospy.logwarn("Service call failed: %s"%e)
                    
        self.prev_status =  status


    def cb_axis(self, msg):

        input_vals = {}
        for attr in self.attrs: # switch to self.axis_attrs!!
            input_vals[attr] = getattr(msg, attr)

        vel_to_pub = Twist()
        twist = vel_to_pub

        for vel_type in self.inputs:
            vel_vec = getattr(twist, vel_type)
            for k, expr in self.inputs[vel_type].items():
                scale = self.scales[vel_type].get(k, 1.0)
                val = eval(expr, {}, input_vals)
                setattr(vel_vec, k, scale * val)

        self.vel_msg = vel_to_pub

        #if self.pub_vel_flag:

            # /cmd_vel publishen
        self.pub_vel.publish(self.vel_msg)



if __name__ == '__main__':

    # Argumente aus ds4_to_tello_twist.launch
    # filename = sys.argv[0]

    try:
        StatusToTello()
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo(" Error ")