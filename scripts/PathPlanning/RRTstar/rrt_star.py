#!/usr/bin/python
# -*- coding: utf-8 -*-
u"""
@brief: Path Planning Sample Code with Randamized Rapidly-Exploring Random Trees (RRT)

@author: AtsushiSakai

@license: MIT

"""


import random
import math
import copy
import numpy as np
from numba import jit


class RRT():
    u"""
    Class for RRT Planning
    """

    def __init__(self, start, goal, obstacleList, randArea,
                 expandDis=0.5, goalSampleRate=50, maxIter=10000):
        u"""
        Setting Parameter

        start:Start Position [x,y]
        goal:Goal Position [x,y]
        obstacleList:obstacle Positions [[x,y,size],...]
        randArea:Ramdom Samping Area [min,max]

        """
        self.start = Node(start[0], start[1])
        self.end = Node(goal[0], goal[1])
        self.minrand = randArea[0]
        self.maxrand = randArea[1]
        self.expandDis = expandDis
        self.goalSampleRate = goalSampleRate
        self.maxIter = maxIter

    def Planning(self, animation=True):
        u"""
        Pathplanning

        animation: flag for animation on or off
        """
        animation = False

        self.nodeList = [self.start]
        for i in range(self.maxIter):
            # Random Sampling
            if random.randint(0, 100) > self.goalSampleRate:
                rnd = [random.uniform(self.minrand, self.maxrand),
                       random.uniform(self.minrand, self.maxrand)]
            else:
                rnd = [self.end.x, self.end.y]

            # Find nearest node
            nind = self.GetNearestListIndex(self.nodeList, rnd)

            # expand tree
            nearestNode = self.nodeList[nind]
            theta = math.atan2(rnd[1] - nearestNode.y, rnd[0] - nearestNode.x)

            newNode = copy.deepcopy(nearestNode)
            newNode.x += self.expandDis * math.cos(theta)
            newNode.y += self.expandDis * math.sin(theta)
            newNode.cost += self.expandDis + \
                self.calc_dist_to_goal(newNode.x, newNode.y)
            newNode.parent = nind
            #  print(newNode.cost)

            if not self.__CollisionCheck(newNode, obstacleList):
                continue

            self.nodeList.append(newNode)

            # check goal
            dx = newNode.x - self.end.x
            dy = newNode.y - self.end.y
            d = math.sqrt(dx * dx + dy * dy)
            if d <= self.expandDis:
                print("Goal!!")
                #  break

            self.rewire(newNode)

            if animation:
                self.DrawGraph(rnd)

        # generate coruse
        lastIndex = self.get_best_last_index()
        path = self.gen_final_course(lastIndex)
        return path

    def get_best_last_index(self):

        disglist = [self.calc_dist_to_goal(
            node.x, node.y) for node in self.nodeList]
        goalinds = [disglist.index(i) for i in disglist if i <= self.expandDis]
        #  print(goalinds)

        mincost = min([self.nodeList[i].cost for i in goalinds])
        for i in goalinds:
            if self.nodeList[i].cost == mincost:
                return i

        return None

    def gen_final_course(self, goalind):
        path = [[self.end.x, self.end.y]]
        while self.nodeList[goalind].parent is not None:
            node = self.nodeList[goalind]
            path.append([node.x, node.y])
            goalind = node.parent
        path.append([self.start.x, self.start.y])
        return path

    def calc_dist_to_goal(self, x, y):
        return np.linalg.norm([x - self.end.x, y - self.end.y])

    def rewire(self, newNode):
        #  print("rewire")
        nnode = len(self.nodeList)
        #  r = 50.0 * math.sqrt((math.log(nnode) / nnode))
        r = self.expandDis * 2.0
        dlist = [math.sqrt((node.x - newNode.x) ** 2 +
                           (node.y - newNode.y) ** 2) for node in self.nodeList]
        nearinds = [dlist.index(i) for i in dlist if i <= r]

        for i in nearinds:
            nearNode = self.nodeList[i]

            dx = nearNode.x - newNode.x
            dy = nearNode.y - newNode.y

            scost = newNode.cost + \
                math.sqrt(dx ** 2 + dy ** 2) + \
                self.calc_dist_to_goal(nearNode.x, nearNode.y)

            if nearNode.cost > scost:
                #  print("rewire")
                nearNode.parent = nnode - 1
                nearNode.cost = scost
                #  input()

    def DrawGraph(self, rnd=None):
        u"""
        Draw Graph
        """
        import matplotlib.pyplot as plt
        plt.clf()
        if rnd is not None:
            plt.plot(rnd[0], rnd[1], "^k")
        for node in self.nodeList:
            if node.parent is not None:
                plt.plot([node.x, self.nodeList[node.parent].x], [
                         node.y, self.nodeList[node.parent].y], "-g")

        plt.plot([ox for (ox, oy, size) in obstacleList], [
                 oy for (ox, oy, size) in obstacleList], "ok", ms=20)
        plt.plot(self.start.x, self.start.y, "xr")
        plt.plot(self.end.x, self.end.y, "xr")
        plt.axis([-2, 15, -2, 15])
        plt.grid(True)
        plt.pause(0.01)

    def GetNearestListIndex(self, nodeList, rnd):
        dlist = [(node.x - rnd[0]) ** 2 + (node.y - rnd[1])
                 ** 2 for node in nodeList]
        minind = dlist.index(min(dlist))

        return minind

    def __CollisionCheck(self, node, obstacleList):

        for (ox, oy, size) in obstacleList:
            dx = ox - node.x
            dy = oy - node.y
            d = math.sqrt(dx * dx + dy * dy)
            if d <= size:
                return False  # collision

        return True  # safe


class Node():
    u"""
    RRT Node
    """

    def __init__(self, x, y):
        self.x = x
        self.y = y
        self.cost = 0.0
        self.parent = None


if __name__ == '__main__':
    import matplotlib.pyplot as plt
    # ====Search Path with RRT====
    obstacleList = [
        (5, 5, 1),
        (3, 6, 2),
        (3, 8, 2),
        (3, 10, 2),
        (7, 5, 2),
        (9, 5, 2)
    ]  # [x,y,size]

    # Set Initial parameters
    rrt = RRT(start=[0, 0], goal=[5, 10],
              randArea=[-2, 15], obstacleList=obstacleList)
    path = rrt.Planning(animation=True)

    # Draw final path
    rrt.DrawGraph()
    plt.plot([x for (x, y) in path], [y for (x, y) in path], '-r')
    plt.grid(True)
    plt.pause(0.01)  # Need for Mac
    plt.show()
