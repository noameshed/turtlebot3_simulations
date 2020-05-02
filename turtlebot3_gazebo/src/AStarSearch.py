# Got help from https://medium.com/@nicholas.w.swift/easy-a-star-pathfinding-7e6689c7f7b2
# https://towardsdatascience.com/a-star-a-search-algorithm-eb495fb156bb

import numpy as np

class Node():
	# Node class for the A* Search

	def __init__(self, parent=None, pos=None):
		self.parent = parent
		self.pos = pos
		self.g = 0
		self.h = 0

	def __eq__(self, other):
		return self.pos == other.pos

def heuristic(grid, goal):
	# Compute the heuristic cost (Euclidean distance to goal)
	H = np.zeros(grid.shape)

	vx = np.linspace(0,11,H.shape[0])
	vy = np.linspace(11,0,H.shape[1])
	xgrid, ygrid = np.meshgrid(vx, vy)

	# make grids of the goal x and y locations (for dist calculations)
	xgoal = goal[0]*np.ones(H.shape)	
	ygoal = goal[1]*np.ones(H.shape)
	
	# compute distance from goal to every location
	H = np.sqrt((xgoal-xgrid)**2 + (ygoal-ygrid)**2)

	return H

def get_adjacent_points(grid, point):
	# Returns a list of the grid coordinates adjacent to the current point
	(w,h) = grid.shape
	(x,y) = point

	# Get the in-bound point coordinates
	left = max(x-1, 0)
	right = min(x+1, w-1)
	top = min(y+1, h-1)
	bot = max(y-1, 0)

	# Store all possible combinations of points
	points = []
	for i in range(left, right+1):
		for j in range(bot, top+1):
			if (i,j) == (x,y):
				continue
			points.append((i,j))

	return points
	
def search(costgrid, start, goal):
	costgrid = (1000*costgrid)

	# Perform an A* search on the provided cost graph 
	# Start at the start point and end at the goal point

	path = []
	# Check if start and goal are already close to each other, then we're done8
	if np.sqrt((start[0]-goal[0])**2 + (start[1]-goal[1])**2) < 0.01:
		return []

	# Create heuristic matrix H (Euclidean distance to goal)
	H = heuristic(costgrid, goal)

	# Initialize open (unsearched) and closed (searched) lists
	startnode = Node(None, start)
	endnode = Node(None, goal)

	tovisit = [startnode]
	visited = []

	# Keep looking until we get to the goal
	while len(tovisit)>0:

		# Get point in open list with smallest cost F=G+H
		cur_node = tovisit[0]
		cur_idx = 0
		for i, node in enumerate(tovisit):
			if node.g+node.h < cur_node.g+cur_node.h:
				cur_node = node
				cur_idx = i

		# Move this point to the closed list
		tovisit.pop(cur_idx)
		visited.append(cur_node)

		# If the current node is the goal, stop
		if cur_node.pos == goal:
			path = []
			cur = cur_node
			while cur is not None:	# backtrack and store locations
				path.append(cur.pos)
				cur = cur.parent

			path.reverse()		# path is from goal to start
			return path

		# Get adjacent points and convert to nodes
		# Current node is the parent
		neighbors = []
		for pos in get_adjacent_points(costgrid, cur_node.pos):
			node = Node(cur_node, pos)	
			neighbors.append(node)	

		# Go through neighbors and assign costs
		for node in neighbors:
			# If already searched, skip
			if node in visited:
				continue

			nodex, nodey = node.pos
			node.h = H[11-nodey, nodex]
			node.g = cur_node.g + costgrid[11-nodey, nodex]

			# If already in open list with a lower cost, skip
			samenode = []
			for n in tovisit:
				if node == n and node.g > n.g:
					samenode.append(n)
				
			if len(samenode) > 0:
				continue
			
			tovisit.append(node)
