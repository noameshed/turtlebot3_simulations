#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist, Pose
import sys, select, os
import math
from gazebo_msgs.msg import ModelStates
import time

def update_pose(data):
    """Callback function which is called when a new message of type Pose is
    received by the subscriber."""
    try:
        robot_pose = data.pose[1]
        position = robot_pose.position
        orient = robot_pose.orientation
        print(position)
        # time.sleep(1)

    except:
        pass
    # self.pose.x = round(self.pose.x, 4)
    # self.pose.y = round(self.pose.y, 4)

if __name__=="__main__":
    rospy.init_node('turtlebot3_controller')
    pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
    print('Controller launched')
    # turtlebot3_model = rospy.get_param("model", "burger") # don't need this line, need if we 
    # want to limit the velocities specific to robot model

    target_linear_vel   = 0.0
    target_angular_vel  = 0.0
    control_linear_vel  = 0.0
    control_angular_vel = 0.0

    pose_subscriber = rospy.Subscriber('/gazebo/model_states', ModelStates, update_pose)

    twist = Twist()
    rate = rospy.Rate(10)
    t = 0
    while not rospy.is_shutdown():
        x_vel = math.sin(t)
        # x_vel = 2
        ang_vel = 1
        twist.linear.x = x_vel; twist.linear.y = 0.0; twist.linear.z = 0.0
        twist.angular.x = 0.0; twist.angular.y = 0.0; twist.angular.z = ang_vel
        pub.publish(twist)
        t+=1
        rate.sleep()