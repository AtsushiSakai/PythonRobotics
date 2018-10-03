"""
Informed RRT* path planning

author: Karan Chawla
        Atsushi Sakai(@Atsushi_twi)

Reference: Informed RRT*: Optimal Sampling-based Path Planning Focused via
Direct Sampling of an Admissible Ellipsoidal Heuristichttps://arxiv.org/pdf/1404.2334.pdf

"""


import random
import numpy as np
import math
import copy
import matplotlib.pyplot as plt

show_animation = True


class InformedRRTStar():

    def __init__(self, start, goal,
                 obstacleList, randArea,
                 expandDis=0.5, goalSampleRate=10, maxIter=200):

        self.start = Node(start[0], start[1])
        self.goal = Node(goal[0], goal[1])
        self.minrand = randArea[0]
        self.maxrand = randArea[1]
        self.expandDis = expandDis
        self.goalSampleRate = goalSampleRate
        self.maxIter = maxIter
        self.obstacleList = obstacleList

    def InformedRRTStarSearch(self, animation=True):

        self.nodeList = [self.start]
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

        for i in range(self.maxIter):
            # Sample space is defined by cBest
            # cMin is the minimum distance between the start point and the goal
            # xCenter is the midpoint between the start and the goal
            # cBest changes when a new path is found

            rnd = self.informed_sample(cBest, cMin, xCenter, C)
            nind = self.getNearestListIndex(self.nodeList, rnd)
            nearestNode = self.nodeList[nind]
            # steer
            theta = math.atan2(rnd[1] - nearestNode.y, rnd[0] - nearestNode.x)
            newNode = self.getNewNode(theta, nind, nearestNode)
            d = self.lineCost(nearestNode, newNode)

            isCollision = self.__CollisionCheck(newNode, self.obstacleList)
            isCollisionEx = self.check_collision_extend(nearestNode, theta, d)

            if isCollision and isCollisionEx:
                nearInds = self.findNearNodes(newNode)
                newNode = self.chooseParent(newNode, nearInds)

                self.nodeList.append(newNode)
                self.rewire(newNode, nearInds)

                if self.isNearGoal(newNode):
                    solutionSet.add(newNode)
                    lastIndex = len(self.nodeList) - 1
                    tempPath = self.getFinalCourse(lastIndex)
                    tempPathLen = self.getPathLen(tempPath)
                    if tempPathLen < pathLen:
                        path = tempPath
                        cBest = tempPathLen

            if animation:
                self.drawGraph(xCenter=xCenter,
                               cBest=cBest, cMin=cMin,
                               etheta=etheta, rnd=rnd)

        return path

    def chooseParent(self, newNode, nearInds):
        if len(nearInds) == 0:
            return newNode

        dList = []
        for i in nearInds:
            dx = newNode.x - self.nodeList[i].x
            dy = newNode.y - self.nodeList[i].y
            d = math.sqrt(dx ** 2 + dy ** 2)
            theta = math.atan2(dy, dx)
            if self.check_collision_extend(self.nodeList[i], theta, d):
                dList.append(self.nodeList[i].cost + d)
            else:
                dList.append(float('inf'))

        minCost = min(dList)
        minInd = nearInds[dList.index(minCost)]

        if minCost == float('inf'):
            print("mincost is inf")
            return newNode

        newNode.cost = minCost
        newNode.parent = minInd

        return newNode

    def findNearNodes(self, newNode):
        nnode = len(self.nodeList)
        r = 50.0 * math.sqrt((math.log(nnode) / nnode))
        dlist = [(node.x - newNode.x) ** 2 +
                 (node.y - newNode.y) ** 2 for node in self.nodeList]
        nearinds = [dlist.index(i) for i in dlist if i <= r ** 2]
        return nearinds

    def informed_sample(self, cMax, cMin, xCenter, C):
        if cMax < float('inf'):
            r = [cMax / 2.0,
                 math.sqrt(cMax**2 - cMin**2) / 2.0,
                 math.sqrt(cMax**2 - cMin**2) / 2.0]
            L = np.diag(r)
            xBall = self.sampleUnitBall()
            rnd = np.dot(np.dot(C, L), xBall) + xCenter
            rnd = [rnd[(0, 0)], rnd[(1, 0)]]
        else:
            rnd = self.sampleFreeSpace()

        return rnd

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

    def getPathLen(self, path):
        pathLen = 0
        for i in range(1, len(path)):
            node1_x = path[i][0]
            node1_y = path[i][1]
            node2_x = path[i - 1][0]
            node2_y = path[i - 1][1]
            pathLen += math.sqrt((node1_x - node2_x) **
                                 2 + (node1_y - node2_y)**2)

        return pathLen

    def lineCost(self, node1, node2):
        return math.sqrt((node1.x - node2.x)**2 + (node1.y - node2.y)**2)

    def getNearestListIndex(self, nodes, rnd):
        dList = [(node.x - rnd[0])**2 +
                 (node.y - rnd[1])**2 for node in nodes]
        minIndex = dList.index(min(dList))
        return minIndex

    def __CollisionCheck(self, newNode, obstacleList):
        for (ox, oy, size) in obstacleList:
            dx = ox - newNode.x
            dy = oy - newNode.y
            d = dx * dx + dy * dy
            if d <= 1.1 * size**2:
                return False  # collision

        return True  # safe

    def getNewNode(self, theta, nind, nearestNode):
        newNode = copy.deepcopy(nearestNode)

        newNode.x += self.expandDis * math.cos(theta)
        newNode.y += self.expandDis * math.sin(theta)

        newNode.cost += self.expandDis
        newNode.parent = nind
        return newNode

    def isNearGoal(self, node):
        d = self.lineCost(node, self.goal)
        if d < self.expandDis:
            return True
        return False

    def rewire(self, newNode, nearInds):
        nnode = len(self.nodeList)
        for i in nearInds:
            nearNode = self.nodeList[i]

            d = math.sqrt((nearNode.x - newNode.x)**2 +
                          (nearNode.y - newNode.y)**2)

            scost = newNode.cost + d

            if nearNode.cost > scost:
                theta = math.atan2(newNode.y - nearNode.y,
                                   newNode.x - nearNode.x)
                if self.check_collision_extend(nearNode, theta, d):
                    nearNode.parent = nnode - 1
                    nearNode.cost = scost

    def check_collision_extend(self, nearNode, theta, d):
        tmpNode = copy.deepcopy(nearNode)

        for i in range(int(d / self.expandDis)):
            tmpNode.x += self.expandDis * math.cos(theta)
            tmpNode.y += self.expandDis * math.sin(theta)
            if not self.__CollisionCheck(tmpNode, self.obstacleList):
                return False

        return True

    def getFinalCourse(self, lastIndex):
        path = [[self.goal.x, self.goal.y]]
        while self.nodeList[lastIndex].parent is not None:
            node = self.nodeList[lastIndex]
            path.append([node.x, node.y])
            lastIndex = node.parent
        path.append([self.start.x, self.start.y])
        return path

    def drawGraph(self, xCenter=None, cBest=None, cMin=None, etheta=None, rnd=None):

        plt.clf()
        if rnd is not None:
            plt.plot(rnd[0], rnd[1], "^k")
            if cBest != float('inf'):
                self.plot_ellipse(xCenter, cBest, cMin, etheta)

        for node in self.nodeList:
            if node.parent is not None:
                if node.x or node.y is not None:
                    plt.plot([node.x, self.nodeList[node.parent].x], [
                        node.y, self.nodeList[node.parent].y], "-g")

        for (ox, oy, size) in self.obstacleList:
            plt.plot(ox, oy, "ok", ms=30 * size)

        plt.plot(self.start.x, self.start.y, "xr")
        plt.plot(self.goal.x, self.goal.y, "xr")
        plt.axis([-2, 15, -2, 15])
        plt.grid(True)
        plt.pause(0.01)

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


class Node():

    def __init__(self, x, y):
        self.x = x
        self.y = y
        self.cost = 0.0
        self.parent = None


def main():
    print("Start informed rrt star planning")

    # create obstacles
    obstacleList = [
        (5, 5, 0.5),
        (9, 6, 1),
        (7, 5, 1),
        (1, 5, 1),
        (3, 6, 1),
        (7, 9, 1)
    ]

    # Set params
    rrt = InformedRRTStar(start=[0, 0], goal=[5, 10],
                          randArea=[-2, 15], obstacleList=obstacleList)
    path = rrt.InformedRRTStarSearch(animation=show_animation)
    print("Done!!")

    # Plot path
    if show_animation:
        rrt.drawGraph()
        plt.plot([x for (x, y) in path], [y for (x, y) in path], '-r')
        plt.grid(True)
        plt.pause(0.01)
        plt.show()


if __name__ == '__main__':
    main()