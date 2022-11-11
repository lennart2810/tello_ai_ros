#!/usr/bin/env python3
#https://answers.gazebosim.org//question/22125/how-to-set-a-models-position-using-gazeboset_model_state-service-in-python/

import rospy 
import rospkg 
from gazebo_msgs.msg import ModelState 
from gazebo_msgs.srv import SetModelState

def main():
    rospy.init_node('set_pose')

    state_msg = ModelState()
    state_msg.model_name = 'quadrotor'
    state_msg.pose.position.x = 1.0
    state_msg.pose.position.y = 2.
    state_msg.pose.position.z = 0.3
    state_msg.pose.orientation.x = 0
    state_msg.pose.orientation.y = 0
    state_msg.pose.orientation.z = 0
    state_msg.pose.orientation.w = 0

    rospy.wait_for_service('/gazebo/set_model_state')
    try:
        set_state = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
        resp = set_state( state_msg )

    except rospy.ServiceException:
        print('fail')

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass













        # models = msg.name
        # quad_index = models.index('quadrotor')
        # #print(quad_index)
        # quad_pose = msg.pose[quad_index]
        # print(quad_pose)
        # #print(type(quad_pose))