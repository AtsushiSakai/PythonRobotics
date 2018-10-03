"""

Path planning code with LQR RRT*

author: AtsushiSakai(@Atsushi_twi)

"""

import sys
sys.path.append("../LQRPlanner/")

import random
import math
import copy
import numpy as np
import matplotlib.pyplot as plt
import LQRplanner

show_animation = True

LQRplanner.show_animation = False

STEP_SIZE = 0.05  # step size of local path
XYTH = 0.5  # [m] acceptance xy distance in final paths


class RRT():
    """
    Class for RRT Planning
    """

    def __init__(self, start, goal, obstacleList, randArea,
                 goalSampleRate=10, maxIter=200):
        """
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
        self.goalSampleRate = goalSampleRate
        self.maxIter = maxIter
        self.obstacleList = obstacleList

    def planning(self, animation=True):
        """
        Pathplanning

        animation: flag for animation on or off
        """

        self.nodeList = [self.start]
        for i in range(self.maxIter):
            rnd = self.get_random_point()
            nind = self.get_nearest_index(self.nodeList, rnd)

            newNode = self.steer(rnd, nind)
            if newNode is None:
                continue

            if self.check_collision(newNode, self.obstacleList):
                nearinds = self.find_near_nodes(newNode)
                newNode = self.choose_parent(newNode, nearinds)
                if newNode is None:
                    continue
                self.nodeList.append(newNode)
                self.rewire(newNode, nearinds)

            if animation and i % 5 == 0:
                self.draw_graph(rnd=rnd)

        # generate coruse
        lastIndex = self.get_best_last_index()
        if lastIndex is None:
            return None
        path = self.gen_final_course(lastIndex)
        return path

    def choose_parent(self, newNode, nearinds):
        if len(nearinds) == 0:
            return newNode

        dlist = []
        for i in nearinds:
            tNode = self.steer(newNode, i)
            if tNode is None:
                continue

            if self.check_collision(tNode, self.obstacleList):
                dlist.append(tNode.cost)
            else:
                dlist.append(float("inf"))

        mincost = min(dlist)
        minind = nearinds[dlist.index(mincost)]

        if mincost == float("inf"):
            print("mincost is inf")
            return newNode

        newNode = self.steer(newNode, minind)

        return newNode

    def pi_2_pi(self, angle):
        return (angle + math.pi) % (2*math.pi) - math.pi

    def sample_path(self, wx, wy, step):

        px, py, clen = [], [], []

        for i in range(len(wx) - 1):

            for t in np.arange(0.0, 1.0, step):
                px.append(t * wx[i + 1] + (1.0 - t) * wx[i])
                py.append(t * wy[i + 1] + (1.0 - t) * wy[i])

        dx = np.diff(px)
        dy = np.diff(py)

        clen = [math.sqrt(idx**2 + idy**2) for (idx, idy) in zip(dx, dy)]

        return px, py, clen

    def steer(self, rnd, nind):

        nearestNode = self.nodeList[nind]

        wx, wy = LQRplanner.LQRplanning(
            nearestNode.x, nearestNode.y, rnd.x, rnd.y)

        px, py, clen = self.sample_path(wx, wy, STEP_SIZE)

        if px is None:
            return None

        newNode = copy.deepcopy(nearestNode)
        newNode.x = px[-1]
        newNode.y = py[-1]

        newNode.path_x = px
        newNode.path_y = py
        newNode.cost += sum([abs(c) for c in clen])
        newNode.parent = nind

        return newNode

    def get_random_point(self):

        if random.randint(0, 100) > self.goalSampleRate:
            rnd = [random.uniform(self.minrand, self.maxrand),
                   random.uniform(self.minrand, self.maxrand),
                   random.uniform(-math.pi, math.pi)
                   ]
        else:  # goal point sampling
            rnd = [self.end.x, self.end.y]

        node = Node(rnd[0], rnd[1])

        return node

    def get_best_last_index(self):
        #  print("get_best_last_index")

        goalinds = []
        for (i, node) in enumerate(self.nodeList):
            if self.calc_dist_to_goal(node.x, node.y) <= XYTH:
                goalinds.append(i)

        if len(goalinds) == 0:
            return None

        mincost = min([self.nodeList[i].cost for i in goalinds])
        for i in goalinds:
            if self.nodeList[i].cost == mincost:
                return i

        return None

    def gen_final_course(self, goalind):
        path = [[self.end.x, self.end.y]]
        while self.nodeList[goalind].parent is not None:
            node = self.nodeList[goalind]
            for (ix, iy) in zip(reversed(node.path_x), reversed(node.path_y)):
                path.append([ix, iy])
            goalind = node.parent
        path.append([self.start.x, self.start.y])
        return path

    def calc_dist_to_goal(self, x, y):
        return np.linalg.norm([x - self.end.x, y - self.end.y])

    def find_near_nodes(self, newNode):
        nnode = len(self.nodeList)
        r = 50.0 * math.sqrt((math.log(nnode) / nnode))
        dlist = [(node.x - newNode.x) ** 2 +
                 (node.y - newNode.y) ** 2
                 for node in self.nodeList]
        nearinds = [dlist.index(i) for i in dlist if i <= r ** 2]
        return nearinds

    def rewire(self, newNode, nearinds):

        nnode = len(self.nodeList)

        for i in nearinds:
            nearNode = self.nodeList[i]
            tNode = self.steer(nearNode, nnode - 1)
            if tNode is None:
                continue

            obstacleOK = self.check_collision(tNode, self.obstacleList)
            imporveCost = nearNode.cost > tNode.cost

            if obstacleOK and imporveCost:
                #  print("rewire")
                self.nodeList[i] = tNode

    def draw_graph(self, rnd=None):
        plt.clf()
        if rnd is not None:
            plt.plot(rnd.x, rnd.y, "^k")

        for node in self.nodeList:
            if node.parent is not None:
                plt.plot(node.path_x, node.path_y, "-g")

        for (ox, oy, size) in self.obstacleList:
            plt.plot(ox, oy, "ok", ms=30 * size)

        plt.plot(self.start.x, self.start.y, "or")
        plt.plot(self.end.x, self.end.y, "or")

        plt.axis([-2, 15, -2, 15])
        plt.grid(True)
        plt.pause(0.01)

    def get_nearest_index(self, nodeList, rnd):
        dlist = [(node.x - rnd.x) ** 2 +
                 (node.y - rnd.y) ** 2
                 for node in nodeList]
        minind = dlist.index(min(dlist))

        return minind

    def check_collision(self, node, obstacleList):

        px = np.array(node.path_x)
        py = np.array(node.path_y)

        for (ox, oy, size) in obstacleList:
            dx = ox - px
            dy = oy - py
            d = dx ** 2 + dy ** 2
            dmin = min(d)
            if dmin <= size ** 2:
                return False  # collision

        return True  # safe


class Node():
    """
    RRT Node
    """

    def __init__(self, x, y):
        self.x = x
        self.y = y
        self.path_x = []
        self.path_y = []
        self.cost = 0.0
        self.parent = None


def main():
    print("Start rrt start planning")

    # ====Search Path with RRT====
    obstacleList = [
        (5, 5, 1),
        (4, 6, 1),
        (4, 7.5, 1),
        (4, 9, 1),
        (6, 5, 1),
        (7, 5, 1)
    ]  # [x,y,size]

    # Set Initial parameters
    start = [0.0, 0.0]
    goal = [6.0, 7.0]

    rrt = RRT(start, goal, randArea=[-2.0, 15.0], obstacleList=obstacleList)
    path = rrt.planning(animation=show_animation)

    # Draw final path
    if show_animation:
        rrt.draw_graph()
        plt.plot([x for (x, y) in path], [y for (x, y) in path], '-r')
        plt.grid(True)
        plt.pause(0.001)
        plt.show()

    print("Done")


if __name__ == '__main__':
    main()
