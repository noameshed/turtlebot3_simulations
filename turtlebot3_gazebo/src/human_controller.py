#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist, Pose
import sys, select, os
import math
from gazebo_msgs.msg import ModelStates, ModelState
import time

def update_pose(data):
    """Callback function which is called when a new message of type Pose is
    received by the subscriber."""
    try:
        all_names = data.name
        p1_ind = all_names.index('person_1')
        p1_pose = data.pose[p1_ind]
        robot_pose = data.pose[1]
        robot_position = robot_pose.position
        robot_orient = robot_pose.orientation
        p1_position = p1_pose.position
        # print(position)

    except:
        time.sleep(1) # probably the publisher not started yet
        pass

if __name__=="__main__":
    rospy.init_node('human_controller')
    pub = rospy.Publisher('/gazebo/set_model_state', ModelState, queue_size=10)
    print('Human Controller launched')

    target_linear_vel   = 0.0
    target_angular_vel  = 0.0
    control_linear_vel  = 0.0
    control_angular_vel = 0.0

    pose_subscriber = rospy.Subscriber('/gazebo/model_states', ModelStates, update_pose)

    p1_state = ModelState()
    p1_state.model_name = 'person_1'
    rate = rospy.Rate(10)
    t = 0
    while not rospy.is_shutdown():
        x_vel = 2*math.sin(t)
        # x_vel = 2
        ang_vel = 0.5
        p1_state.twist.linear.x = x_vel
        p1_state.twist.angular.x = ang_vel
        pub.publish(p1_state)
        t+=1
        rate.sleep()