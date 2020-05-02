#!/usr/bin/env python
import rospy
import geometry_msgs.msg
from geometry_msgs.msg import Twist, Pose
import sys, select, os
import math
from gazebo_msgs.msg import ModelStates, ModelState, LinkState
import time
import numpy as np

class RobotController():
    """Robot navigation model following Sisbot et al's 'A Human Aware Mobile
    Robot Motion Planner' """
    def __init__(self):

        rospy.init_node('turtlebot3_burger')
        self.pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)

        self.pose_subscriber = rospy.Subscriber('/gazebo/model_states', ModelStates, self.update_pose)

        # Initialize robot state
        # self.robot_state = ModelState()
        # self.robot_state.model_name = 'turtlebot3_burger'


        self.people = ['person_1', 'person_2', 'person_3', 'person_4']
        self.rate = rospy.Rate(10)
        self.final_goal = (-5, 0)
        self.human_poses = None
        self.human_positions = None

        self.pose = None

        # Boundaries of the room
        self.x_bound = [-7.5, 7.5]
        self.y_bound = [-2.5, 2.5]

        self.msg = geometry_msgs.msg.Twist()
        self.reset()

        print('Robot Controller Launched')


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

            self.human_positions = [p1_position, p2_position, p3_position, p4_position]
            self.human_poses = [p1_pose, p2_pose, p3_pose, p4_pose]

            robot_ind = all_names.index('turtlebot3_burger')
            robot_pose = data.pose[robot_ind]
            self.pose = robot_pose
            self.goal = (-3, 0)

            # print('robot', robot_pose)

        except Exception as e:
            print(e)
            time.sleep(1) # probably the publisher not started yet
            pass

    def reset(self):
        # Reset the robot turtle (make it stop moving)
        self.msg.linear.x = 0
    	self.msg.linear.y = 0
    	self.msg.linear.z = 0
    	self.msg.angular.x = 0
    	self.msg.angular.y = 0
    	self.msg.angular.z = 0

    def cost_map(self, sigma, n):
        """Creates a Gaussian cost map for a human in the room based on sigma. all
        humans in the environment have the same cost map, just centered at
        different locations. n is the size of the cost map (it is (2n+1)x(2n+n))"""
        x, y = np.meshgrid(np.linspace(-n,n,2*n+1), np.linspace(-n,n,2*n+1))
        d = np.sqrt(x*x+y*y)
        const = 1/(sigma*math.sqrt(2*math.pi))
        map = const * np.exp(-0.5*(d/sigma)**2)

        return map

    def get_distance(self, position, goal):
        position = np.array([position.x, position.y])
        return np.linalg.norm(position-goal)

    def get_angle(self, position, goal):
        """Computes angle between robot and goal"""
        position = np.array([position.x, position.y])
        diff = position - goal
        angle = math.atan2(diff[1], diff[0])
        return angle

    def go_to_goal(self, goal, tolerance):
        """Get the robot to go to the defined goal,
            goal should be a [x, y] array type."""
        # tolerance = 0.05

        print("STARTING ROBOT NAVIGATION")
        goal = np.array(goal)
        # curr_time = time.time()
        dist = self.get_distance(self.pose.position, goal)
        while dist>tolerance:

            # Calculate the distance and angle between robot and goal
            dist = self.get_distance(self.pose.position, goal)
            angle = self.get_angle(self.pose.position, goal)
            theta_robot = self.pose.orientation.z
            print(theta_robot, dist)

            # Set robot velocity
            self.msg.linear.x = 0.25   # Move faster when farther from the goal
            # self.msg.angular.z = 2*(angle - theta_robot)

            self.pub.publish(self.msg)
            self.rate.sleep()

        # Reset pose once goal is reached
        self.reset()
        self.pub.publish(self.msg)

        print("robot done moving to goal")

    def control_robot(self, door):
        while self.pose == None:
            time.sleep(1)
        self.go_to_goal(self.final_goal, tolerance=0.05)
        return

if __name__=="__main__":
    try:
        door_pose = [0, 0]
        xlim = [-7.5, 7.5]
        ylim = [-2.5, 2.5]
        rcontrol = RobotController()
        rcontrol.control_robot(door_pose)

    except rospy.ROSInterruptException:
        pass

    # hum_cost = cost_map(1.0, 5)

    # twist = Twist()
    # rate = rospy.Rate(10)
    # t = 0
    # while not rospy.is_shutdown():
    #     # x_vel = math.sin(t)
    #     x_vel = 2
    #     ang_vel = math.sin(t)
    #     twist.linear.x = x_vel; twist.linear.y = 0.0; twist.linear.z = 0.0
    #     twist.angular.x = 0.0; twist.angular.y = 0.0; twist.angular.z = ang_vel
    #     pub.publish(twist)
    #     t+=1
    #     rate.sleep()
