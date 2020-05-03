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
import tf
from matplotlib import pyplot as plt
import threading

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
        self.path = []
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
        # Initialize empty room map
        grid_block_size = 0.25 # Map detail is to the closest 1/4 block
        num_xblocks = int((1./grid_block_size) * (self.x_bound[1]-self.x_bound[0]))
        num_yblocks = int((1./grid_block_size) * (self.y_bound[1]-self.y_bound[0]))
        self.vx = np.linspace(self.x_bound[0], self.x_bound[1], num_xblocks)
        self.vy = np.linspace(self.y_bound[0], self.y_bound[1], num_yblocks)
        self.map_x, self.map_y = np.meshgrid(self.vx, self.vy)
        self.cost_grid = np.zeros(self.map_x.shape)

    def set_cost_grid_walls(self):
        grid_block_size = 0.25 # Map detail is to the closest 1/4 block
        # wall_cost = np.inf
        wall_cost = 1000
        num_xblocks = int((1./grid_block_size) * (self.x_bound[1]-self.x_bound[0]))
        num_yblocks = int((1./grid_block_size) * (self.y_bound[1]-self.y_bound[0]))
        # Set wall costs to infinity
        self.cost_grid[0,:] = wall_cost*np.ones(self.cost_grid[0,:].shape)
        self.cost_grid[num_yblocks-1,:] = wall_cost*np.ones(self.cost_grid[num_yblocks-1,:].shape)
        self.cost_grid[:,0] = wall_cost*np.ones(self.cost_grid[:,0].shape)
        self.cost_grid[:,num_xblocks-1] = wall_cost*np.ones(self.cost_grid[:,num_xblocks-1].shape)

        # Walls around door have cost of infinity
        wall_len = 2.5 - (0.7/2)
        num_blocks_door = int(np.floor((1./grid_block_size) * wall_len))
        door_top = num_blocks_door+1
        door_bot = num_yblocks-num_blocks_door-1
        door_left = num_xblocks/2 - 1
        door_right = num_xblocks/2 + 1

        self.cost_grid[:door_top,door_left:door_right] = \
            wall_cost*np.ones(self.cost_grid[:door_top,door_left:door_right].shape)
        self.cost_grid[door_bot:,door_left:door_right] = \
            wall_cost*np.ones(self.cost_grid[door_bot:,door_left:door_right].shape)

        # Door goes to indices 9 and 11 in x direction (up-down)


    def update_map(self, sigma, n, mult):
        """Updates the cost map for the room based on the humans' current
        locations. Each human has a Gaussian safety cost map based on sigma
        and centered at their position.
        n is the size of the individual human's personal space (it is (2n+1)x(2n+n))
        mult is the scaling of the human safety cost. Higher mult would cause planner to 
        get stuck more often"""

        # Compute cost for a single person
        l = 2*n+1
        x, y = np.meshgrid(np.linspace(-n,n,l), np.linspace(-n,n,l))
        d = np.sqrt(x*x+y*y)
        const = 1./(sigma*math.sqrt(2*math.pi))
        gauss = mult * const * np.exp(-0.5*(d/sigma)**2)

        # Update the cost grid based on each person's location
        self.reset_cost_grid()
        for pos in self.human_positions:
            gauss = mult * const * np.exp(-0.5*(d/sigma)**2)
            # Don't need to worry about walls because adding a cost to np.inf
            # is still np.inf
            pos_idx = self.pose_to_gridpoint(pos)
            x1, y1 = pos_idx - n
            x2, y2 = pos_idx + n

            max_x, max_y = self.cost_grid.shape
            # Handle cost when people are near walls
            if x1 < 0:
                gauss = gauss[abs(x1):,:]
                x1 = 0
            elif x2 > max_x-1:
                gauss = gauss[:l-(x2 - max_x+1),:]
                x2 = max_x-1
            if y1 < 0:
                gauss = gauss[:,abs(y1):]
                y1 = 0
            elif y2 > max_y-1:
                gauss = gauss[:,:l-(y2 - max_y+1)]
                y2 = max_y-1


            # Update cost grid
            self.cost_grid[x1:x2+1, y1:y2+1] = gauss
        self.set_cost_grid_walls()

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

        print("STARTING ROBOT NAVIGATION")
        goal = np.array(goal)

        dist = self.get_distance(self.pose.position, goal)
        M = np.zeros((2, 2))
        vel_multiplier = 1
        ang_multiplier = 1
        angle = self.get_angle(self.pose.position, goal)
        r_th = self.pose.orientation
        euler = tf.transformations.euler_from_quaternion((r_th.x,r_th.y,r_th.z,r_th.w))

        # while abs(angle-euler[2])>0.05:
        #     # print(euler)
        #     # print(angle)
        #     self.msg.linear.x = 0
        #     self.msg.angular.z = (angle - euler[2])
        #     self.pub.publish(self.msg)
        #     angle = self.get_angle(self.pose.position, goal)
        #     r_th = self.pose.orientation
        #     euler = tf.transformations.euler_from_quaternion((r_th.x,r_th.y,r_th.z,r_th.w))
        #     self.rate.sleep()


        # while dist>tolerance:
        #     self.msg.linear.x = vel_multiplier*dist
        #     self.msg.angular.z = 0
        #     self.pub.publish(self.msg)

        #     self.rate.sleep()
        #     dist = self.get_distance(self.pose.position, goal)


        while dist>tolerance:

            # Calculate the distance and angle between robot and goal
            dist = self.get_distance(self.pose.position, goal)
            angle = self.get_angle(self.pose.position, goal)

            # theta_robot = self.pose.orientation.z

            # Robot motion controller
            # del_x = vel_multiplier*(goal[0]-self.pose.position.x)
            # del_y = vel_multiplier*(goal[1]-self.pose.position.y)
            # M[0, 0] = np.cos(theta_robot)
            # M[0, 1] = np.sin(theta_robot)
            # M[1, 0] = -np.sin(theta_robot)/0.1
            # M[1, 1] = np.cos(theta_robot)/0.1
            # control = np.dot(M,np.array([[del_x], [del_y]]))
            # self.msg.linear.x = control[0]
            # self.msg.angular.z = control[1]

            r_th = self.pose.orientation
            euler = tf.transformations.euler_from_quaternion((r_th.x,r_th.y,r_th.z,r_th.w))

            # Set robot velocity
            self.msg.linear.x = min(dist, 0.5)   # Move faster when farther from the goal
            self.msg.angular.z = 2*(angle - euler[2])
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
        for i,y in enumerate(self.vy):
            if abs(pose.y - y) < ydist:
                ydist = abs(pose.y - y)
                yidx = i

        return np.array([yidx,xidx])

    def control_robot(self, final_goal):
        """Controls the robots movement along the A* path to the goal"""
        # Wait for robot initialization
        while self.pose == None:
            time.sleep(1)
        self.final_goal = final_goal
        mult = 50 #default safety cost scale
        n = 5 #default safety gaussian size

        # Initialize A* map
        startpoint = self.pose_to_gridpoint(self.pose.position)
        endpoint = self.pose_to_gridpoint(self.final_goal.position)
        self.update_map(sigma=1, n=n, mult=mult)
        # print(self.cost_grid)
        # plt.imshow(self.cost_grid, cmap='hot', interpolation='nearest')
        # plt.show()
        path = search(self.cost_grid, self.map_x, self.map_y, startpoint, endpoint, {})
        # print(path)
        # found_goal = False
        while len(path)>0:
            mult = 10 #default safety cost scale
            n = 5 #default safety gaussian size
            p = path.pop(0)
            self.go_to_goal(p, tolerance=0.125)
            if len(path)==0:
                print("Robot reached final goal")
                break 
            # Update the map and grid costs with human positions
            self.update_map(sigma=1, n=n, mult=mult)
            # print('COST GRID')
            # print(self.cost_grid[:,27:33])
            # print(self.cost_grid)
            # plt.imshow(self.cost_grid, cmap='hot', interpolation='nearest')
            # plt.show()
            startpoint = self.pose_to_gridpoint(self.pose.position)
            path_dict = {'p': None}
            t = threading.Thread(target=search, args=(self.cost_grid, self.map_x, self.map_y, startpoint, endpoint, path_dict))
            # path = search(self.cost_grid, self.map_x, self.map_y, startpoint, endpoint)
            t.start()
            t.join(timeout=2) #replan
            path = path_dict['p']
            while not path: # timed out, need to replan
                mult = mult/2
                print("stuck! replanning with smaller cost")
                self.update_map(sigma=1, n=n, mult=mult)
                path_dict = {'p': None}
                t = threading.Thread(target=search, args=(self.cost_grid, self.map_x, self.map_y, startpoint, endpoint, path_dict))
                t.start()
                t.join(timeout=2) #replan
                path = path_dict['p']
            # print("THREAD PATH IS: ", path_dict['p'])


if __name__=="__main__":
    try:
        door_pose = [0, 0]
        robot_goal = Pose()
        robot_goal.position.x = 2
        robot_goal.position.y = 2
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
