# importing dependencies
import math
import random
import numpy as np 
import matplotlib.pyplot as plt 

show_animation = True

class RRT:
	"""RRT class that performs Path Planning (Rapidly Exploring Random Trees)"""
	class Node:
		"""Node class to express different nodes in the trees"""
		def __init__(self, x, y):
			"""initialize parameters of a node"""
			self.x = x
			self.y = y
			self.path_x = []
			self.path_y = []
			self.parent = None

	def __init__(self, start, goal, obstacle_list, min_max_points, 
				 max_len = 3.0,
				 path_resolution = 0.5,
				 goal_sample_rate = 5,
				 max_iter = 500):
		"""initialize the parameters of the RRT class"""
		self.start = self.Node(start[0], start[1])  # starting position
		self.end = self.Node(goal[0], goal[1])  # goal position
		self.min_point = min_max_points[0]  # min point in the graph
		self.max_point = min_max_points[1]  # max point in the graph
		self.obstacle_list = obstacle_list  # list of all the obstacles
		self.max_len = max_len  # max distance to travel from a node
		self.path_resolution = path_resolution  # path resolution
		self.goal_sample_rate = goal_sample_rate  # range within which the goal node will be considered
		self.max_iter = max_iter  # maximum number of times to run the loop for rrt
		self.node_list = []  # list for adding all the valid nodes

	def find_path(self, animation = True):
		"""Function the find the path between start and end positions"""
		self.node_list = [self.start]

		for i in range(self.max_iter):
			random_node = self.get_random_node()  # getting a random node
			nearest_node_ind = self.get_nearest_node_index(self.node_list, random_node)   # get index of the nearest node
			nearest_node = self.node_list[nearest_node_ind]  # get the node at that index

			new_node = self.move(nearest_node, random_node, self.max_len)  # moving in the direction of the random node from the nearest node

			if self.check_collision(new_node, self.obstacle_list):  # if the new node doesn't collide with any obstacle then append it to the node list
				self.node_list.append(new_node)

			if animation and (i % 5 == 0):  # update the graph every 5 iteration
				self.draw_graph(random_node)

			if self.calc_dist_to_goal(self.node_list[-1].x, self.node_list[-1].y) <= self.max_len:   # checking the distance of the last appended node to the goal
				final_node = self.move(self.node_list[-1], self.end, self.max_len)  # if distance is less than a threshold then move in that direction
				if self.check_collision(final_node, self.obstacle_list):   # check if the final node doesn't collide with the obstacle
					return self.generate_final_course(len(self.node_list) - 1)  # if not, then the final path has been found, and generate the final path

			if animation and (i % 5 == 0):   # draw the graph
				self.draw_graph(random_node)

		return None

	def move(self, from_node, to_node, max_len = float("inf")):
		"""Function to move from one node to another"""
		new_node = self.Node(from_node.x, from_node.y)
		d, angle = self.calc_dist_and_angle(new_node, to_node)   # calculating dist and angle between the 2 nodes

		new_node.path_x = [new_node.x]  # adding the x co-ordinate of starting node to it's path
		new_node.path_y = [new_node.y]  # adding the y co-ordinate of starting node to it's path

		if d < max_len:   # dist is less than max_len then max_len will the distance that will be travelled
			max_len = d

		n_expand = math.floor(max_len / self.path_resolution)  # number of times to run the loop for generating path between 2 nodes

		for _ in range(n_expand):
			# moving in steps of length path_resolution with the anlge
			new_node.x += self.path_resolution * math.cos(angle)
			new_node.y += self.path_resolution * math.sin(angle) 

			# appending the point the path of the node
			new_node.path_x.append(new_node.x)
			new_node.path_y.append(new_node.y)

		# calculating new distane of the new_node and the node to be reached
		d, _ = self.calc_dist_and_angle(new_node, to_node)

		# if distance is less than path_resolution, then append the node to reached to the path
		# this means the the node has been reached
		if d <= self.path_resolution:
			new_node.path_x.append(to_node.x)
			new_node.path_y.append(to_node.y)

			new_node.x = to_node.x
			new_node.y = to_node.y

		# the node from where we arrived to the new_node is now the parent of the new_node
		new_node.parent = from_node

		return new_node

	def generate_final_course(self, goal_ind):
		"""This Function generates the final path between start and goal"""
		path = [[self.end.x, self.end.y]]  # first appending the goal to the path
		node = self.node_list[goal_ind]

		# this while loop runs until a node is reached such that it doesn't have any parent
		# which is the starting node
		# hence the path will be generated in reverse order
		while node.parent is not None:  
			path.append([node.x, node.y])
			node = node.parent

		# appending the start node to the path
		path.append([node.x, node.y])

		return path

	def calc_dist_to_goal(self, x, y):
		"""Function to calculate the distance between current node and goal"""
		dx = x - self.end.x
		dy = y - self.end.y 

		return math.hypot(dx, dy)

	def get_random_node(self):
		"""Function to generate a random node"""
		if random.randint(0, 100) > self.goal_sample_rate:
			return self.Node(random.uniform(self.min_point, self.max_point), random.uniform(self.min_point, self.max_point))
		else:
			return self.Node(self.end.x, self.end.y)

	def draw_graph(self, rnd = None):
		"""Function to plot the graph"""
		plt.clf()

		# stopping the simulation using esc key
		plt.gcf().canvas.mpl_connect(
			'key_release_event',
			lambda event : [exit(0) if event.key == 'escape' else None])

		# plotting the random node
		if rnd is not None:
			plt.plot(rnd.x, rnd.y, "^k")
		
		# plotting all the nodes in the node_list
		for node in self.node_list:
			if node.parent:
				plt.plot(node.path_x, node.path_y, "-g")

		# plotting all the obstacles
		for cx, cy, size in self.obstacle_list:
			self.plot_circle(cx, cy, size)

		plt.plot(self.start.x, self.start.y, "xr")
		plt.plot(self.end.x, self.end.y, "xr")
		plt.axis("equal")
		plt.axis([-2, 15, -2, 15])
		plt.grid(True)
		plt.pause(0.01)

	@staticmethod
	def plot_circle(x, y, size, color = "-b"):
		"""Function to plot a circle"""
		deg = list(range(0, 360, 5))
		deg.append(0)

		x1 = [x + size * math.cos(np.deg2rad(d)) for d in deg]
		y1 = [y + size * math.sin(np.deg2rad(d)) for d in deg]

		plt.plot(x1, y1, color)

	@staticmethod
	def get_nearest_node_index(node_list, random_node):
		"""Function to find the index of the node nearest to the random_node"""
		dlist = [(node.x - random_node.x)**2 + (node.y - random_node.y)**2 for node in node_list]
		return dlist.index(min(dlist))

	@staticmethod
	def check_collision(node, obstacleList):
		"""Function to check if the node collides with an obstacle"""
		if node is None:
			return False 

		for cx, cy, size in obstacleList:
			# calculate the difference between the circle centers and the nodes
			dx_list = [cx - x for x in node.path_x]
			dy_list = [cy - y for y in node.path_y]

			d_list = [dx**2 + dy**2 for (dx, dy) in zip(dx_list, dy_list)]

			# if the minimum of all the distances within d_list is less than the radius of the circle, then there is a collision
			if min(d_list) <= size**2:
				return False  # return False because of collision

		return True  # no collision

	@staticmethod
	def calc_dist_and_angle(from_node, to_node):
		"""Calculate distance between 2 nodes"""
		dx = to_node.x - from_node.x
		dy = to_node.y - from_node.y

		return math.hypot(dx, dy), math.atan2(dy, dx)


def main(gx = 6.0, gy = 8.0):
	"""Main Function"""

	# obstacle list
	obstacleList = [(5, 5, 1), (3, 6, 2), (3, 8, 2), (3, 10, 2), (7, 5, 2), (9, 5, 2), (8, 10, 1)]  # (x, y, radius)

	# set initial parameters
	rrt = RRT(
		start = [1.0, 4.0],
		goal = [gx, gy],
		min_max_points = [-2, 15],
		obstacle_list = obstacleList)

	# planning the path
	path = rrt.find_path(animation = show_animation)

	if path is None:
		print("Path Cannot Be Found.")
	else:
		print("Path Found.")

	# draw final path
	if show_animation:
		rrt.draw_graph()
		plt.plot([x for x, y in path], [y for x, y in path], '-r')
		plt.grid(True)
		plt.pause(0.01)
		plt.show()


if __name__ == "__main__":
	main()

