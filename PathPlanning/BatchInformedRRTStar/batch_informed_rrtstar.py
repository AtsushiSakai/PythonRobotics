"""
Batch Informed Trees based path planning

author: Karan Chawla(@karanchawla)

Reference: https://arxiv.org/abs/1405.5848
"""

import random
import numpy as np 
import copy 
import operator 
import math
import matplotlib.pyplot as plt 

show_animation = True 

class Tree(object):

	def __init__(self, start, lowerLimit, upperLimit, resolution):
		self.vertices = dict()
		vertex_id = self.gridCoordinateToNodeId(start)
		self.vertices[vid] = []
		self.edges = []
		self.start = start
		self.lowerLimit = lowerLimit
		self.upperLimit = upperLimit
		for idx in range(len(lowerLimit)):
			self.num_cells[idx] = np.ceil((upperLimit[idx] - lowerLimit[idx])/resolution)

	def getRootId(self):
		return 0

	def addVertex(self, vertex):
		vertex_id = self.gridCoordinateToNodeId(vertex)
		self.vertices[vertex_id] = []
		return vertex_id

	def addEdge(self, v, x):
		if (v, x) not in self.edges:
			self.edges.append((v,x))
		self.vertices[v].append(x)
		self.vertices[x].append(v)

	def gridCoordinateToNodeId(self, coord):
		nodeId = 0
		for i in range(len(coord) - 1, -1, -1):
			product = 1
			for j in range(0 , i):
				product *= product * self.num_cells[j]
			node_id = node_id + coord[i] * product
		return node_id


class Node():

    def __init__(self, x, y):
        self.x = x
        self.y = y
        self.cost = 0.0
        self.parent = None

class BITStar():

	def __init__(self, start, goal, 
				 obstacleList, randArea, eta=2.0,
				 expandDis=0.5, goalSampleRate=10, maxIter=200):
		self.start = Node(start[0], start[1])
		self.goal = Node(goal[0], goal[1])
		self.minrand = randArea[0]
		self.maxrand = randArea[1]
		self.expandDis = expandDis
		self.goalSampleRate = goalSampleRate
		self.maxIter = maxIter
		self.obstacleList = obstacleList
		self.vertex_queue = []
		self.edge_queue = []
		self.samples = dict()
		self.g_scores = dict()
		self.f_scores = dict()
		self.r = float('inf')
		self.eta = eta # tunable parameter
		self.unit_ball_measure = 1
		self.old_vertices = []

	def plan(self, animation=True):

		self.nodeList = [self.start]
		plan = None 
		iterations = 0 
		# max length we expect to find in our 'informed' sample space, starts as infinite
		cBest = float('inf')
		pathLen = float('inf')
		solutionSet = set()
		path = None

		# Computing the sampling space
		cMin = math.sqrt(pow(self.start.x - self.goal.x, 2) +
						 pow(self.start.y - self.goal.y, 2))
		xCenter = np.matrix([[(self.start.x + self.goal.x) / 2.0],
                             [(self.start.y + self.goal.y) / 2.0], [0]])
		a1 = np.matrix([[(self.goal.x - self.start.x) / cMin],
                        [(self.goal.y - self.start.y) / cMin], [0]])
		etheta = math.atan2(a1[1], a1[0])
		# first column of idenity matrix transposed
		id1_t = np.matrix([1.0, 0.0, 0.0])
		M = np.dot(a1, id1_t)
		U, S, Vh = np.linalg.svd(M, 1, 1)
		C = np.dot(np.dot(U, np.diag(
			[1.0, 1.0, np.linalg.det(U) * np.linalg.det(np.transpose(Vh))])), Vh)

		foundGoal = False
		# run until done
		while (iterations < self.maxIter):
			if len(self.vertex_queue) == 0 and len(self.edge_queue) == 0:
				samples = self.informedSample(100, cBest, cMin, xCenter, C)
				# prune the tree




			if animation:
				self.drawGraph(xCenter=xCenter, cBest=cBest,
							   cMin=cMin, etheta=etheta, samples=samples)

			iterations += 1

		return plan

	# def expandVertex(self, vertex):

	# def prune(self, c):

	def radius(self, q):
		dim = len(start) #dimensions
		space_measure = self.minrand * self.maxrand # volume of the space
		
		min_radius = self.eta * 2.0 * pow((1.0 + 1.0/dim) * 
					 (space_measure/self.unit_ball_measure), 1.0/dim) 
		return min_radius * pow(numpy.log(q)/q, 1/dim)

	# Return the closest sample 
	# def getNearestSample(self):

	# Sample free space confined in the radius of ball R
	def informedSample(self, m, cMax, cMin, xCenter, C):
		samples = []
		if cMax < float('inf'):
			for i in range(m):
				r = [cMax / 2.0,
	                 math.sqrt(cMax**2 - cMin**2) / 2.0,
	                 math.sqrt(cMax**2 - cMin**2) / 2.0]
				L = np.diag(r)
				xBall = self.sampleUnitBall()
				rnd = np.dot(np.dot(C, L), xBall) + xCenter
				rnd = [rnd[(0, 0)], rnd[(1, 0)]]
				samples.append(rnd)
		else: 
			for i in range(m):
				rnd = self.sampleFreeSpace()
				samples.append(rnd)
		return samples

	# Sample point in a unit ball
	def sampleUnitBall(self):
		a = random.random()
		b = random.random()

		if b < a:
			a, b = b, a

		sample = (b * math.cos(2 * math.pi * a / b),
				  b * math.sin(2 * math.pi * a / b))
		return np.array([[sample[0]], [sample[1]], [0]])

	def sampleFreeSpace(self):
		if random.randint(0, 100) > self.goalSampleRate:
			rnd = [random.uniform(self.minrand, self.maxrand),
				   random.uniform(self.minrand, self.maxrand)]
		else:
			rnd = [self.goal.x, self.goal.y]

		return rnd

	# def bestVertexQueueValue(self):

	# def bestEdgeQueueValue(self):

	# def bestInEdgeQueue(self):

	# def bestInVertexQueue(self):

	# def updateGraph(self):

	def drawGraph(self, xCenter=None, cBest=None, cMin=None, etheta=None, samples=None):
		print("Plotting Graph")
		plt.clf()
		for rnd in samples:
			if rnd is not None:
				plt.plot(rnd[0], rnd[1], "^k")
				if cBest != float('inf'):
					self.plot_ellipse(xCenter, cBest, cMin, etheta)

		# for node in self.nodeList:
		# 	if node.parent is not None:
		# 		if node.x or node.y is not None:
		# 			plt.plot([node.x, self.nodeList[node.parent].x], [
		# 					node.y, self.nodeList[node.parent].y], "-g")

		for (ox, oy, size) in self.obstacleList:
			plt.plot(ox, oy, "ok", ms=30 * size)

		plt.plot(self.start.x, self.start.y, "xr")
		plt.plot(self.goal.x, self.goal.y, "xr")
		plt.axis([-2, 15, -2, 15])
		plt.grid(True)
		plt.pause(5)

	def plot_ellipse(self, xCenter, cBest, cMin, etheta):

		a = math.sqrt(cBest**2 - cMin**2) / 2.0
		b = cBest / 2.0
		angle = math.pi / 2.0 - etheta
		cx = xCenter[0]
		cy = xCenter[1]

		t = np.arange(0, 2 * math.pi + 0.1, 0.1)
		x = [a * math.cos(it) for it in t]
		y = [b * math.sin(it) for it in t]
		R = np.matrix([[math.cos(angle), math.sin(angle)],
                       [-math.sin(angle), math.cos(angle)]])
		fx = R * np.matrix([x, y])
		px = np.array(fx[0, :] + cx).flatten()
		py = np.array(fx[1, :] + cy).flatten()
		plt.plot(cx, cy, "xc")
		plt.plot(px, py, "--c")

def main():
	print("Starting Batch Informed Trees Star planning")
	obstacleList = [
        (5, 5, 0.5),
        (9, 6, 1),
        (7, 5, 1),
        (1, 5, 1),
        (3, 6, 1),
        (7, 9, 1)
    ]

	bitStar = BITStar(start=[0, 0], goal=[5, 10],
				randArea=[-2, 15], obstacleList=obstacleList)
	path = bitStar.plan(animation=show_animation)
	print("Done")

if __name__ == '__main__':
    main()
