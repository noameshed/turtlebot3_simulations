#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist, Pose
import sys, select, os
import math
from gazebo_msgs.msg import ModelStates, ModelState
import time

class HumanController():
    """Human navigation based on social forces model"""
    def __init__(self):
        rospy.init_node('human_controller')
        self.human_speed = 15 #typically 1.5 m/s
        self.pub = rospy.Publisher('/gazebo/set_model_state', ModelState, queue_size=10)
        self.pose_subscriber = rospy.Subscriber('/gazebo/model_states', ModelStates, self.update_pose)

        self.p1_state = ModelState()
        self.p1_state.model_name = 'person_1'

        self.p2_state = ModelState()
        self.p2_state.model_name = 'person_2'

        self.p3_state = ModelState()
        self.p3_state.model_name = 'person_3'

        self.p4_state = ModelState()
        self.p4_state.model_name = 'person_4'
        self.rate = rospy.Rate(10)

        self.p1_pose = self.p2_pose = self.p3_pose = self.p4_pose = None
        self.p1_position = self.p2_position = self.p3_position = self.p4_position = None


        print('Human Controller launched initialized')

    def update_pose(self, data):
        """Callback function which is called when a new message of type Pose is
        received by the subscriber. This function is to unpack the poses of objects"""
        try:
            all_names = data.name
            # print(all_names)
            p1_ind = all_names.index('person_1')
            self.p1_pose = data.pose[p1_ind]
            self.p1_position = self.p1_pose.position

            p2_ind = all_names.index('person_2')
            self.p2_pose = data.pose[p2_ind]
            self.p2_position = self.p2_pose.position

            p3_ind = all_names.index('person_3')
            self.p3_pose = data.pose[p3_ind]
            self.p3_position = self.p3_pose.position

            p4_ind = all_names.index('person_4')
            self.p4_pose = data.pose[p4_ind]
            self.p4_position = self.p4_pose.position

            robot_ind = all_names.index('turtlebot3_burger')
            robot_pose = data.pose[robot_ind]
            self.robot_position = robot_pose.position

        except Exception as e:
            print(e)
            time.sleep(1) # probably the publisher not started yet
            pass

    def get_distance(self, position, goal):
        return math.sqrt((position.x-goal[0])**2+(position.y-goal[1])**2)


    def go_to_goal(self, goal):
        """Get the person to go to the defined goal,
            goal should be a [x, y] array type. 
            Try on person 1 first, probably need to add the specific person as argument"""
        tolerance = 0.05
        while self.p1_position==None:
            time.sleep(1)
        print("STARTING HUMAN NAVIGATION")
        while self.get_distance(self.p1_position, goal)>tolerance:
            self.p1_state.pose = self.p1_pose
            x_diff = goal[0]-self.p1_position.x
            y_diff = goal[1]-self.p1_position.y
            #normalize to max vel
            scale = self.human_speed/math.sqrt(x_diff**2+y_diff**2)
            self.p1_state.twist.linear.x = scale*x_diff
            self.p1_state.twist.linear.y = scale*y_diff
            self.pub.publish(self.p1_state)
    
    def control_human(self):
        """Main function to control all humans"""
        raise NotImplementedError


if __name__=="__main__":
    try:
        hcontrol = HumanController()
        door_pose = [0,0]
        hcontrol.go_to_goal(door_pose)
    except rospy.ROSInterruptException:
        pass

    # t = 0
    # while not rospy.is_shutdown():
    #     x_vel = 2*math.sin(t)
    #     # x_vel = 2
    #     ang_vel = 0.5
    #     p1_state.twist.linear.x = x_vel
    #     p1_state.twist.angular.x = ang_vel
    #     pub.publish(p1_state)
    #     t+=1
    #     rate.sleep()
