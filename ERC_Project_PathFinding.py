#import files
import matplotlib.pyplot as plt
import math
import random
from shapely.geometry import Polygon, LineString


#global variables

xticks, yticks = [(float)(x/2) for x in range(0, 9)], [(float)(x/2) for x in range(0, 9)]
#obstacle_coords1 = ([(0,0), (0,4), (0.25,3.75), (0.25,2.5), (1,2.5), (1,3), (1.5,3), (1.5,2.75), (1.25,2.75), (1.25,2.25), (0.25,2.25), (0.25,0.25), (1,0.25), (1,1), (1.5,1), (1.5,0.75), (1.25,0.75), (1.25,0.25), (2.5,0.25), (2.5,1.5), (2.75,1.5), (2.75,0.25), (3.75,0.25), (4,0), (0,0), (0,4)])
#obstacle_coords2 = ([(0,4), (0.25,3.75), (2.75,3.75), (2.75,2.5), (3.75,2.5), (3.75,0.25), (4,0), (4,4)])

obstacle_coords1 = [(0,4),(0.25,3.75),(0.25,0.25),(1,0.25),(1,3),(2,3),(2,2.25),(1.5,2.25),(1.5,0.25),(3.75,0.25),(4,0),(0,0)]
obstacle_coords2 = [(0,4),(0.25,3.75),(2.5,3.75),(2.5,2),(3,2),(3,3.75),(3.75,3.75),(3.75,0),(4,0),(4,4)]

start_coords = [0.5, 0.5]
end_coords = [3.5, 3.5]

limit = 5000
LI_NODES = []
LI_CONNECT = []

fig, axs = plt.subplots()
axs.fill([x[0] for x in obstacle_coords1], [y[1] for y in obstacle_coords1], alpha = 1, fc = '#000000', ec = 'none')
axs.fill([x[0] for x in obstacle_coords2], [y[1] for y in obstacle_coords2], alpha = 1, fc = '#000000', ec = 'none')

obs1 = Polygon(obstacle_coords1)
obs2 = Polygon(obstacle_coords2)

isJoined = False

plt.title('ERC Path Planning', fontdict = {'fontsize':20})
#class of Nodes which contains following methods
#1. function to check LOS to objective


class Node:
	def __init__(self, xcoord, ycoord, weight, marker, size, connect):
		self.xcoord = xcoord
		self.ycoord = ycoord
		self.weight = weight
		self.marker = marker
		self.size = size
		self.connect = connect
		plt.plot([xcoord], [ycoord], self.marker , markersize = self.size)

	def LOSCheck(self):
		global isJoined
		line = LineString([[self.xcoord, self.ycoord], [end_coords[0], end_coords[1]]])
		if not (line.intersects(obs1) or line.intersects(obs2)):
			isJoined = True
			plt.plot([self.xcoord, end_coords[0]], [self.ycoord, end_coords[1]], 'go-')


#function to find the distance between random point and nodes
def dist(Node, x, y):
	return math.sqrt((Node.xcoord-x)**2 + (Node.ycoord-y)**2)**0.5

#def connectNode(n1, n2):
#	plt.plot([n1.xcoord, n2.xcoord], [n1.ycoord, n2.ycoord], 'ro-', markersize = 5)

#predefined Nodes
start = Node(start_coords[0], start_coords[1], 0, 'bo', 20, 0)
goal = Node(end_coords[0], end_coords[1], 0, 'bo', 20, 0)
LI_NODES.append(start)
cv = 0

while (cv < limit and (not isJoined)):
	plt.pause(0.0001)
	x = random.random()*4
	y = random.random()*4
	min = float('inf')
	joining_node = None

	for node in LI_NODES:
		line = LineString([[x, y], [node.xcoord, node.ycoord]])
		if (dist(node, x, y) <= 1 and not (line.intersects(obs1) or line.intersects(obs2))) and dist(node, x, y) < min:
			min = dist(node, x, y)
			joining_node = node

	if joining_node:
		plt.plot([x, joining_node.xcoord], [y, joining_node.ycoord], 'yo-', markersize = 3)
		LI_NODES.append(Node(x, y, dist(joining_node, x, y)+joining_node.weight, 'yo', 3, LI_NODES.index(joining_node)))
		LI_NODES[-1].LOSCheck()

	cv += 1

#node1 = LI_NODES[-1]
#node2 = LI_NODES[node1.connect]
#while node2 != LI_NODES[0]:
#	connectNode(node1, node2)
#	node1, node2 = node2, LI_NODES[node1.connect]
	#node1 = node2
	#node2 = LI_NODES[node1.connect]

#connectNode(node1, node2)

LI_CONNECT.append(len(LI_NODES)-1)
while LI_NODES[LI_CONNECT[-1]].connect != 0:
	LI_CONNECT.append(LI_NODES[LI_CONNECT[-1]].connect)

LI_CONNECT.append(0)

plt.plot([LI_NODES[LI_CONNECT[i]].xcoord for i in range(len(LI_CONNECT))], [LI_NODES[LI_CONNECT[i]].ycoord for i in range(len(LI_CONNECT))], 'go-')


plt.show()