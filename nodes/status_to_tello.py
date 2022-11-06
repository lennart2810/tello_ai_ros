#!/usr/bin/env python3

# -- ds4_to_tello_twist.py --
# Version vom 30.01.2022 by LF
# ClassDefinition StatusToTelloTwist
# ----------------------------


import sys
import rospy
#import subprocess
from geometry_msgs.msg import Twist
from std_msgs.msg import String, Bool
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

        self.pub_cmd_flag = True

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

        self.req_msg_template = tello_serviceRequest(False, False, False, False, False, False, False, False, False, False)

        # /cmd_vel
        #self.vel_msg = Twist
        self.pub_vel_flag = True
        self.pub_vel = rospy.Publisher('cmd_vel', Twist, queue_size=1)

        # /set_feedback
        self.feedback = Feedback()
        self.feedback.set_led = True
        self.feedback.set_led_flash = True
        self.pub_feedback = rospy.Publisher('set_feedback', Feedback, queue_size=1)

        # /control_flag
        self.ai_control_flag = False
        self.pub_control_flag = rospy.Publisher('ai_control_flag', Bool, queue_size=1)


    def cb_status(self, msg):

        # safety feature (enabling switch) for autonomous drone control (ai mode)
        ai_flag = msg.touch0.active
        if ai_flag is not self.ai_control_flag: # check for change in flag
            self.pub_control_flag.publish(ai_flag) # touchpad acitvated --> ai mode (True), else rc (False) 

        if ai_flag == False:
            # Axis
            self.cb_axis(msg)

        # Buttons
        self.cb_button(msg)

        # /set_feedback <-- Batterieanzeige (Controller)
        if msg.battery_percentage <= 0.375:
            self.feedback.led_flash_on = 0.3
            self.feedback.led_flash_off = 0.3
        elif msg.battery_percentage <= 0.25:
            self.feedback.led_flash_on = 0.1
            self.feedback.led_flash_off = 0.1
        else:
            self.feedback.led_flash_off = 0


    def cb_button(self, msg):
        # detect change in button status
        for attr in self.button_attrs:
            if getattr(msg, attr) is not getattr(self.prev_status, attr): # flag in button status detected
                if getattr(msg, attr) == True: # positive flag in one button status detected
                    
                    # prepare service call
                    rospy.wait_for_service('tello_button_service')
                    request = rospy.ServiceProxy('tello_button_service', tello_service)
                    req_msg = self.req_msg_template

                    try:
                        if attr == 'button_dpad_up':
                            req_msg.up = True
                        elif attr == 'button_dpad_down':
                            req_msg.down = True
                        elif attr == 'button_dpad_left':
                            req_msg.left = True
                        elif attr == 'button_dpad_right':
                            req_msg.right = True

                        elif attr == 'button_cross':
                            req_msg.cross = True
                        elif attr == 'button_circle':
                            req_msg.circle = True
                        elif attr == 'button_triangle':
                            req_msg.triangle = True
                        elif attr == 'button_square':
                            req_msg.square = True

                        # buttons without service call
                        else:
                            if attr == 'button_trackpad':
                                rospy.loginfo("Battery: %s %%", ( msg.battery_percentage * 100))
                                rospy.loginfo("USB: %s",  msg.plug_usb)
                                #self.pub_cmd_flag = not self.pub_cmd_flag
                            break

                        response = request(req_msg)
                        rospy.loginfo('service response' + str(response))
                        break

                    except rospy.ServiceException as e:
                        rospy.logwarn("Service call failed: %s"%e)
                    
        self.prev_status =  msg


    def cb_axis(self, msg):

        # wenn ai aktiviert ist,  soll von hier aus nicht gepublisht werden
        
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

        self.vel_msg = vel_to_pub # notwendig???
        self.pub_vel.publish(vel_to_pub)



if __name__ == '__main__':

    # Argumente aus ds4_to_tello_twist.launch
    # filename = sys.argv[0]

    try:
        StatusToTello()
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo(" Error ")