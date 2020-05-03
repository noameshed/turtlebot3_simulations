#!/usr/bin/env python
import rospy
import geometry_msgs.msg
from geometry_msgs.msg import Twist, Pose
import sys, select, os
import math
from gazebo_msgs.msg import ModelStates, ModelState, LinkState
import time
import numpy as np
from AStarSearch import search

class RobotController():
    """Robot navigation model following Sisbot et al's 'A Human Aware Mobile
    Robot Motion Planner' """
    def __init__(self):

        rospy.init_node('turtlebot3_burger')
        self.pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)

        self.pose_subscriber = rospy.Subscriber('/gazebo/model_states', ModelStates, self.update_pose)

        # Initialize human states
        self.people = ['person_1', 'person_2', 'person_3', 'person_4']
        self.rate = rospy.Rate(30)
        self.human_poses = None
        self.human_positions = None

        # Robot current and goal locations
        self.pose = None
        self.final_goal = Pose()

        # Initialize the room cost map
        self.cost_grid = []
        self.x_bound = [-7.5, 7.5]
        self.y_bound = [-2.5, 2.5]
        self.reset_cost_grid()

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

    def reset_cost_grid(self):
        # Initialize room map
        grid_block_size = 0.25 # Map detail is to the closest 1/4 block
        num_xblocks = int((1./grid_block_size) * (self.x_bound[1]-self.x_bound[0]))
        num_yblocks = int((1./grid_block_size) * (self.y_bound[1]-self.y_bound[0]))
        self.vx = np.linspace(self.x_bound[0], self.x_bound[1], num_xblocks)
        self.vy = np.linspace(self.y_bound[0], self.y_bound[1], num_yblocks)
        self.map_x, self.map_y = np.meshgrid(self.vx, self.vy)
        self.cost_grid = np.zeros(self.map_x.shape)

        # Set wall costs to infinity
        self.cost_grid[0,:] = np.inf*np.ones(self.cost_grid[0,:].shape)
        self.cost_grid[num_yblocks-1,:] = np.inf*np.ones(self.cost_grid[num_yblocks-1,:].shape)
        self.cost_grid[:,0] = np.inf*np.ones(self.cost_grid[:,0].shape)
        self.cost_grid[:,num_xblocks-1] = np.inf*np.ones(self.cost_grid[:,num_xblocks-1].shape)
        # Walls around door
        wall_len = 2.5 - (0.7/2)
        num_blocks_door = int(np.floor((1./grid_block_size) * wall_len))
        self.cost_grid[:num_blocks_door,num_xblocks/2] = \
            np.inf*np.ones(self.cost_grid[:num_blocks_door,num_xblocks/2].shape)
        self.cost_grid[num_yblocks-num_blocks_door:,num_xblocks/2] = \
            np.inf*np.ones(self.cost_grid[num_yblocks-num_blocks_door:,num_xblocks/2].shape)


    def update_map(self, sigma, n):
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
        diff = goal - position
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
        print(self.pose.position, goal)
        M = np.zeros((2, 2))
        vel_multiplier = 1
        while dist>tolerance:

            # Calculate the distance and angle between robot and goal
            dist = self.get_distance(self.pose.position, goal)
            angle = self.get_angle(self.pose.position, goal)
            theta_robot = self.pose.orientation.z
            
            # Set robot velocity
            self.msg.linear.x = 0.3+min(0.25, dist+0.1)   # Move faster when farther from the goal
            self.msg.angular.z = 0.1+(angle - theta_robot)
            self.pub.publish(self.msg)
            self.rate.sleep()

        # Reset pose once goal is reached
        self.reset()
        self.pub.publish(self.msg)

        print("robot done moving to goal: ", goal)

    def pose_to_gridpoint(self, pose):
        """Computes the indices of the gridpoint which the position is closest to"""
        xdist = np.inf
        xidx = 0
        for i,x in enumerate(self.vx):
            if abs(pose.x - x) < xdist:
                xdist = abs(pose.x - x)
                xidx = i

        ydist = np.inf
        yidx = 0
        print('VX and VY LENGTHS')
        print(len(self.vx))
        print(len(self.vy))
        for i,y in enumerate(self.vy):
            if abs(pose.y - y) < ydist:
                ydist = abs(pose.y - y)
                yidx = i

        return (yidx,xidx)


    def control_robot(self, final_goal):
        """Controls the robots movement along the A* path to the goal"""
        # Wait for robot initialization
        while self.pose == None:
            time.sleep(1)
        self.final_goal = final_goal

        # Initialize A* map
        startpoint = self.pose_to_gridpoint(self.pose.position)
        endpoint = self.pose_to_gridpoint(self.final_goal.position)

        path = search(self.cost_grid, self.map_x, self.map_y, startpoint, endpoint)
        # print(path)

        for p in path:
            self.go_to_goal(p, tolerance=0.15)


if __name__=="__main__":
    try:
        door_pose = [0, 0]
        robot_goal = Pose()
        robot_goal.position.x = -2
        robot_goal.position.y = 0
        robot_goal.position.z = 0
        xlim = [-7.5, 7.5]
        ylim = [-2.5, 2.5]
        rcontrol = RobotController()
        rcontrol.control_robot(robot_goal)

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
