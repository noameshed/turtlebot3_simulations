#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist, Pose
import sys, select, os
import math
from gazebo_msgs.msg import ModelStates, ModelState, LinkState
import time
import threading

class HumanController():
    """Human navigation based on social forces model"""
    def __init__(self):
        rospy.init_node('human_controller')
        self.human_speed = 1.5 #typically 1.5 m/s
        self.pub = rospy.Publisher('/gazebo/set_model_state', ModelState, queue_size=10)
        # self.pub = rospy.Publisher('/gazebo/set_link_state', LinkState, queue_size=10)
        self.pose_subscriber = rospy.Subscriber('/gazebo/model_states', ModelStates, self.update_pose)

        p1_state = ModelState()
        # self.p1_state = LinkState()
        p1_state.model_name = 'person_1'

        p2_state = ModelState()
        p2_state.model_name = 'person_2'

        p3_state = ModelState()
        p3_state.model_name = 'person_3'

        p4_state = ModelState()
        p4_state.model_name = 'person_4'

        self.rate = rospy.Rate(10)
        self.states = [p1_state, p2_state, p3_state, p4_state]
        self.final_goals = [(7, -2), (7, 2), (-7, -2), (-7, 2)]
        self.poses = None
        self.positions = None

        print('Human Controller launched initialized')

    def update_pose(self, data):
        """Callback function which is called when a new message of type Pose is
        received by the subscriber. This function is to unpack the poses of objects"""
        try:
            all_names = data.name
            # print(all_names)
            p1_ind = all_names.index('person_1')
            p1_pose = data.pose[p1_ind]
            p1_position = p1_pose.position

            p2_ind = all_names.index('person_2')
            p2_pose = data.pose[p2_ind]
            p2_position = p2_pose.position

            p3_ind = all_names.index('person_3')
            p3_pose = data.pose[p3_ind]
            p3_position = p3_pose.position

            p4_ind = all_names.index('person_4')
            p4_pose = data.pose[p4_ind]
            p4_position = p4_pose.position

            robot_ind = all_names.index('turtlebot3_burger')
            robot_pose = data.pose[robot_ind]
            self.robot_position = robot_pose.position
            self.positions = [p1_position, p2_position, p3_position, p4_position]
            self.poses = [p1_pose, p2_pose, p3_pose, p4_pose]

        except Exception as e:
            print(e)
            time.sleep(1) # probably the publisher not started yet
            pass

    def get_distance(self, position, goal):
        return math.sqrt((position.x-goal[0])**2+(position.y-goal[1])**2)


    def go_to_goal(self, goal, person):
        """Get the person to go to the defined goal,
            goal should be a [x, y] array type. 
            person index in [0, 1, 2, 3]
            Try on person 1 first, probably need to add the specific person as argument"""
        tolerance = 0.05

        print("STARTING HUMAN NAVIGATION")
        # curr_time = time.time()
        while self.get_distance(self.positions[person], goal)>tolerance:
            # prev_time = curr_time
            # curr_time = time.time()
            # dur = curr_time-prev_time
            x_diff = goal[0]-self.positions[person].x
            y_diff = goal[1]-self.positions[person].y
            # print(self.states[person])
            # print(self.poses[person])
            self.states[person].pose.orientation = self.poses[person].orientation
            #normalize to max vel
            scale = self.human_speed/math.sqrt(x_diff**2+y_diff**2)
            self.states[person].pose.position.x = self.positions[person].x+scale*x_diff*0.01
            self.states[person].pose.position.y = self.positions[person].y+scale*y_diff*0.01
            self.states[person].twist.linear.x = scale*x_diff
            self.states[person].twist.linear.y = scale*y_diff
            self.pub.publish(self.states[person])
        print("done moving to goal")
    
    def control_human(self, goal):
        """Main function to control all humans"""
        while self.poses==None:
            time.sleep(1)
        threads = []
        for i in range(4):
            x = threading.Thread(target=self.go_to_goal, args=(goal, i))
            threads.append(x)
            x.start()
            time.sleep(2) # move 2 secs apart
        threads2 = []
        for i, t in enumerate(threads):
            t.join()
            y = threading.Thread(target=self.go_to_goal, args=(self.final_goals[i], i))
            threads2.append(y)
            y.start()
        for t in threads2:
            t.join()



if __name__=="__main__":
    try:
        hcontrol = HumanController()
        door_pose = [0,0]
        hcontrol.control_human(door_pose)
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
