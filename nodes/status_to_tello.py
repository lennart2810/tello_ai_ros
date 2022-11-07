#!/usr/bin/env python3

#import sys
import rospy
from std_msgs.msg import Bool
from ds4_driver.msg import Status
from geometry_msgs.msg import Twist
from tello_ai_ros.srv import tello_service, tello_serviceRequest, tello_serviceResponse


class StatusToTello(object):
    def __init__(self):

        rospy.init_node('status_to_tello')
        
        # load params from status_to_tello.yaml file
        self.inputs = rospy.get_param('~inputs')
        self.scales = rospy.get_param('~scales')

        self.axis = rospy.get_param('~axis')
        self.axis_attrs = list(self.axis.values())

        self.button = rospy.get_param('~button')
        self.button_attrs = list(self.button.values())

        # /status
        self.prev_status = Status()
        rospy.Subscriber('status', Status, self.cb_status, queue_size=1)

        # /cmd_vel/rc
        self.pub_vel = rospy.Publisher('cmd_vel/rc', Twist, queue_size=1)

        # /control_flag
        self.ai_control_flag = False
        self.pub_control_flag = rospy.Publisher('ai_control_flag', Bool, queue_size=1)


    def cb_status(self, msg):

        # safety feature (enabling switch (touchpad)) for autonomous drone control
        ai_flag = msg.touch0.active
        self.pub_control_flag.publish(ai_flag)
    
        # publish cmd_vel/rc in remote control mode
        if not ai_flag:
            self.cb_axis(msg)

        self.cb_button(msg)
    

    def cb_axis(self, msg):
        
        # get input values for each axis attribute
        input_vals = {}
        for attr in self.axis_attrs:
            input_vals[attr] = getattr(msg, attr)

        cmd_vel_rc = Twist()

        for vel_type in self.inputs:
            vel_vec = getattr(cmd_vel_rc, vel_type)
            for k, expr in self.inputs[vel_type].items():
                scale = self.scales[vel_type].get(k, 1.0)
                val = eval(expr, {}, input_vals)
                setattr(vel_vec, k, scale * val)

        self.pub_vel.publish(cmd_vel_rc)


    def cb_button(self, msg):

        # detect change in button status
        for attr in self.button_attrs:
            if getattr(msg, attr) is not getattr(self.prev_status, attr): # flag in button status detected
                if getattr(msg, attr) == True: # positive flag in one button status detected

                    # buttons without service call
                    if attr == 'button_trackpad':
                        rospy.loginfo("Battery: %s %%", ( msg.battery_percentage * 100))
                        rospy.loginfo("USB: %s",  msg.plug_usb)
                        break
                    
                    # prepare service call
                    #rospy.wait_for_service('tello_button_service', 1.0)
                    request = rospy.ServiceProxy('tello_button_service', tello_service)

                    try:
                        response = request(tello_serviceRequest(attr))
                        rospy.loginfo('service ' + str(response))
                        break

                    except rospy.ServiceException as e:
                        rospy.logwarn("Service call failed: %s"%e)
                    
        self.prev_status =  msg



if __name__ == '__main__':

    try:
        StatusToTello()
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo(" Error ")