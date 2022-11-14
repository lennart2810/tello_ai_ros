#!/usr/bin/env python3

import sys
import rospy
from ds4_driver.msg import Feedback
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool
from tello_ai_ros.msg import object_position


class TelloControllerNode(object):
    def __init__(self, view, detector):
 
        # init node, publisher and subscriber
        rospy.init_node('tello_controller_node', anonymous=False)
        rospy.Subscriber('cmd_vel/rc', Twist, self.rc_cb, queue_size=1)
        rospy.Subscriber('object_position', object_position, self.object_position_cb, queue_size=1)

        rospy.Subscriber('ai_control_flag', Bool, self.ai_control_flag_callback)
        self.ai_control_flag = True

        self.pub_vel = rospy.Publisher('cmd_vel', Twist, queue_size=1)

        # /set_feedback
        self.feedback = Feedback()
        self.feedback.set_led = True
        self.pub_feedback = rospy.Publisher('set_feedback', Feedback, queue_size=1)
        
        if detector == 'pose':
             goal_params = rospy.get_param('~pose_goal')
             self.goals = (goal_params['x'], goal_params['y'], goal_params['l'])
             gain_params = rospy.get_param('~pose_gain')
             self.gains = (gain_params['x'], gain_params['y'], gain_params['l'])
        elif detector == 'face':
             goal_params = rospy.get_param('~goal')
             self.goals = (goal_params['x'], goal_params['y'], goal_params['z'])
        elif detector == 'gazebo':
             goal_params = rospy.get_param('~pose_goal')
             self.goals = (goal_params['x'], goal_params['y'], goal_params['z'])


    def rc_cb(self, msg):
        if not self.ai_control_flag:
            self.pub_vel.publish(msg)


    def object_position_cb(self, msg):
        if self.ai_control_flag:

            if msg.detected:
                cmd_vel = self.compute_cmd_vel_ai(msg)
                self.feedback.led_r = 0; self.feedback.led_g = 1; self.feedback.led_b = 0

            else:
                # rotate to find object
                cmd_vel = Twist()
                cmd_vel.angular.z = int(10)
                self.feedback.led_r = 1; self.feedback.led_g = 0; self.feedback.led_b = 0

            self.pub_vel.publish(cmd_vel)
        
        else:
            self.feedback.led_r = 0; self.feedback.led_g = 0; self.feedback.led_b = 1

        self.pub_feedback.publish(self.feedback)

    
    def ai_control_flag_callback(self, msg):
        if msg.data:
            self.ai_control_flag = True
        else:
            self.ai_control_flag = False


    def compute_cmd_vel_ai(self, msg):

        e_x = msg.x - self.goals[0]
        e_y = msg.y- self.goals[1]
        e_z = msg.l - self.goals[2]
        
        # p_x = self.gains[0]
        # p_y = self.gains[1]
        # p_z = self.gains[2]

        cmd_vel = Twist()
        cmd_vel.linear.x = int(-e_z * self.gains[0])  # x.drohne --> vor und zurück in der ebene; hier wird abstand zum gesicht geregelt
        cmd_vel.linear.y = int(e_x * self.gains[1]) # y.drohne --> links rechts in der ebene; positionierung in der vertikalen achse 
        cmd_vel.linear.z = int(e_y * self.gains[2]) # z.drone --> hoch und runter entland der hochachse
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