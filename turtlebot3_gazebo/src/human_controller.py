#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist, Pose
import sys, select, os
import math
from gazebo_msgs.msg import ModelStates, ModelState, LinkState
import time
import threading
import numpy as np

class HumanController():
    """Human navigation based on social forces model"""
    def __init__(self):
        rospy.init_node('human_controller')
        self.human_speed = 0.8 #typically 1.5 m/s
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
        self.x_bound = [-7.5, 7.5]
        self.y_bound = [-2.5, 2.5]

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
            robot_twist = data.twist[robot_ind]
            self.robot_linvel = robot_twist.linear.x
            self.robot_angvel = robot_twist.angular.z
            self.robot_position = robot_pose.position
            self.positions = [p1_position, p2_position, p3_position, p4_position]
            self.poses = [p1_pose, p2_pose, p3_pose, p4_pose]

        except Exception as e:
            print(e)
            time.sleep(1) # probably the publisher not started yet
            pass

    def get_distance(self, position, goal):
        position = np.array([position.x, position.y])
        return np.linalg.norm(position-goal)


    def go_to_goal(self, goal, person, tolerance):
        """Get the person to go to the defined goal,
            goal should be a [x, y] array type.
            person index in [0, 1, 2, 3]
            Try on person 1 first, probably need to add the specific person as argument"""
        # tolerance = 0.05

        print("STARTING HUMAN NAVIGATION")
        goal = np.array(goal)
        # curr_time = time.time()
        while self.get_distance(self.positions[person], goal)>tolerance:
            # get potential gradient and set velocity
            curr_position = np.array([self.positions[person].x, self.positions[person].y])
            grad = self.get_point_force(curr_position, goal, person)
            xVel = -grad[0]
            yVel = -grad[1]
            self.states[person].pose.orientation = self.poses[person].orientation
            #normalize to max vel
            scale = self.human_speed/np.linalg.norm(np.array([xVel, yVel]))
            self.states[person].pose.position.x = self.positions[person].x+scale*xVel*0.01
            self.states[person].pose.position.y = self.positions[person].y+scale*yVel*0.01
            self.states[person].twist.linear.x = scale*xVel
            self.states[person].twist.linear.y = scale*yVel
            self.pub.publish(self.states[person])
        print("done moving to goal")

    def control_human(self, goal):
        """Main function to control all humans"""
        while self.poses==None:
            time.sleep(1)
        threads = []
        for i in range(4):
            x = threading.Thread(target=self.go_to_goal, args=(goal, i, 0.3))
            threads.append(x)
            x.start()
            time.sleep(1) # move 2 secs apart
        threads2 = []
        for i, t in enumerate(threads):
            t.join()
            y = threading.Thread(target=self.go_to_goal, args=(self.final_goals[i], i, 0.05))
            threads2.append(y)
            y.start()
        for t in threads2:
            t.join()

    def get_point_force(self, point, goal, person):
        """At the given point in the map, get the gradient of the
        potential function. point and goal should both be 1x2 np arrays"""
        wall_rep_const = 5
        human_rep_const = 15 #stay further away from humans than to walls
        att_const = 5
        human_radius = 1 # influence range of other humans
        wall_radius = 0.5 # influence range of the wall
        # the robot repulsion parameters should be dynamic

        robot_rep_const = 50*abs(self.robot_linvel+self.robot_angvel)+1
        # print(robot_rep_const)
        robot_radius = 1.5
        #get components of gradients and then sum
        att = att_const*(point-goal) #attraction to the goal
        human_rep = np.array([0, 0])
        for i in range(4):
            h_position = self.positions[i]
            dist_to_person = np.linalg.norm(point-[h_position.x, h_position.y])
            if i!=person and dist_to_person<human_radius: #only consider other humans
                nabla = ([h_position.x, h_position.y]-point)/dist_to_person
                human_rep = human_rep+human_rep_const*nabla*(1/dist_to_person)
        # robot repulsion
        dist_to_robot = np.linalg.norm(point-[self.robot_position.x, self.robot_position.y])
        if dist_to_robot<robot_radius:
            nabla = ([self.robot_position.x, self.robot_position.y]-point)/dist_to_robot
            robot_rep = robot_rep_const*nabla*(1/dist_to_robot)
        else:
            robot_rep = [0,0]
        # get gradients due to walls
        wall_rep = np.array([0,0])
        if abs(point[0])<wall_radius and (point[1]>0.3 or point[1]<-0.3): #close to the middle wall
            wall_rep = wall_rep+np.array([wall_rep_const*(-1/point[0]), 0])
        elif abs(point[0]-(-7.5))<wall_radius:
            wall_rep = wall_rep+np.array([wall_rep_const*(1/(-7.5-point[0])), 0])
        elif abs(point[0]-7.5)<wall_radius:
            wall_rep = wall_rep+np.array([wall_rep_const*(1/(7.5-point[0])), 0])

        if abs(point[1]-(-2.5))<wall_radius: #top and bottom wall
            wall_rep = wall_rep+np.array([0, wall_rep_const*(1/(-2.5-point[1]))])
        elif abs(point[1]-2.5)<wall_radius: #top and bottom wall
            wall_rep = wall_rep+np.array([0, wall_rep_const*(1/(2.5-point[1]))])
        # print("att: ", att)
        # print("wall rep: ", wall_rep)
        # print("human_rep: ", human_rep)
        try:
            final_grad =att+wall_rep+human_rep+robot_rep
        except:
            print("att: ", att)
            print("wall rep: ", wall_rep)
            print("human_rep: ", human_rep)

        return final_grad

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
