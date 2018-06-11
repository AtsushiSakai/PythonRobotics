"""
Batch Informed Trees based path planning

author: Karan Chawla(@karanchawla)

Reference: https://arxiv.org/abs/1405.5848
"""

import random
import numpy as np 
from math import cos, sin, atan2, pi 
import copy 
import matplotlib.pyplot as plt 

show_animation = True 

class Node():

    def __init__(self, x, y):
        self.x = x
        self.y = y
        self.cost = 0.0
        self.parent = None

class BITStar():

	def __init__(self, start, goal, 
				 obstacleList, randArea, 
				 expandDis=0.5, goalSampleRate=10, maxIter=200, eta):
		self.start = Node(start[0], start[1])
		self.goal = Node(goal[0], goal[1])
		self.minrand = randArea[0]
		self.maxrand = randArea[1]
		self.expandDis = expandDis
		self.goalSampleRate = goalSampleRate
		self.maxIter = maxIter
		self.obstacleList = obstacleList
		self.samples = dict()
		self.g_scores = dict()
		self.f_scores = dict()
		self.r = float('inf')
		self.eta = eta # tunable parameter
		self.unit_ball_measure = #TODO

	def plan(self, animation=True):

		self.nodeList = [self.start]







	def expandVertex(self, vertex):

	def prune(self, c):

	def radius(self, q):
		dim = len(start) #dimensions
		space_measure = self.minrand * self.maxrand # volume of the space
		
		min_radius = self.eta * 2.0 * pow((1.0 + 1.0/dim) * 
					 (space_measure/self.unit_ball_measure), 1.0/dim) 
		return min_radius * pow(numpy.log(q)/q, 1/dim)

	# Return the closest sample 
	def getNearestSample(self):

	# Sample free space confined in the radius of ball R
	def sample(self, m, cMax):

	# Sample point in a unit ball
	def sampleUnitBall(self, m):

	def bestVertexQueueValue(self):

	def bestEdgeQueueValue(self):

	def bestInEdgeQueue(self):

	def bestInVertexQueue(self):

	def updateGraph(self):