"""
Path Planning Sample Code with RRT for car like robot.

author: AtsushiSakai(@Atsushi_twi)
"""
import copy
import sys

import matplotlib.pyplot as plt
import numpy as np

sys.path.append("../ReedsSheppPath/")

import reeds_shepp_path_planning


show_animation = True
STEP_SIZE = 0.1
curvature = 1.0


class RRT():
    """
    Class for RRT Planning
    """

    def __init__(self, start, goal, obstacleList, randArea,
                 goalSampleRate=10, maxIter=400):
        """
        Setting Parameter

        start:Start Position [x,y]
        goal:Goal Position [x,y]
        obstacleList:obstacle Positions [[x,y,size],...]
        randArea:Ramdom Samping Area [min,max]

        """
        self.start = Node(start[0], start[1], start[2])
        self.end = Node(goal[0], goal[1], goal[2])
        self.minrand = randArea[0]
        self.maxrand = randArea[1]
        self.goalSampleRate = goalSampleRate
        self.maxIter = maxIter
        self.obstacleList = obstacleList

    def Planning(self, animation=True):
        """
        Pathplanning

        animation: flag for animation on or off
        """

        self.nodeList = [self.start]
        for i in range(self.maxIter):
            rnd = self.get_random_point()
            nind = self.GetNearestListIndex(self.nodeList, rnd)

            newNode = self.steer(rnd, nind)
            if newNode is None:
                continue

            if self.CollisionCheck(newNode, self.obstacleList):
                nearinds = self.find_near_nodes(newNode)
                newNode = self.choose_parent(newNode, nearinds)
                if newNode is None:
                    continue
                self.nodeList.append(newNode)
                self.rewire(newNode, nearinds)

            if animation and i % 5 == 0:
                self.DrawGraph(rnd=rnd)

        # generate coruse
        lastIndex = self.get_best_last_index()
        if lastIndex is None:
            return None
        path = self.gen_final_course(lastIndex)
        return path

    def choose_parent(self, newNode, nearinds):
        if len(nearinds) == 0:
            return newNode

        min_cost = np.inf
        min_idx = 0
        for i in nearinds:
            tNode = self.steer(newNode, i)
            if tNode is None:
                continue

            if self.CollisionCheck(tNode, self.obstacleList):
                if tNode.cost < min_cost:
                    min_cost = tNode.cost
                    min_idx = i

        if min_cost == np.inf:
            print("min_cost is inf")
            return newNode

        newNode = self.steer(newNode, min_idx)

        return newNode

    def pi_2_pi(self, angle):
        return (angle + np.pi) % (2*np.pi) - np.pi

    def steer(self, rnd, nind):

        nearestNode = self.nodeList[nind]

        px, py, pyaw, mode, clen = reeds_shepp_path_planning.reeds_shepp_path_planning(
            nearestNode.x, nearestNode.y, nearestNode.yaw, rnd.x, rnd.y,
            rnd.yaw, curvature, STEP_SIZE)

        if px is None:
            return None

        newNode = copy.deepcopy(nearestNode)
        newNode.x = px[-1]
        newNode.y = py[-1]
        newNode.yaw = pyaw[-1]

        newNode.path_x = px
        newNode.path_y = py
        newNode.path_yaw = pyaw
        newNode.cost += sum([abs(c) for c in clen])
        newNode.parent = nind

        return newNode

    def get_random_point(self):

        if np.random.randint(0, 100) > self.goalSampleRate:
            rnd = [np.random.uniform(self.minrand, self.maxrand),
                   np.random.uniform(self.minrand, self.maxrand),
                   np.random.uniform(-np.pi, np.pi)
                   ]
        else:  # goal point sampling
            rnd = [self.end.x, self.end.y, self.end.yaw]

        node = Node(rnd[0], rnd[1], rnd[2])

        return node

    def get_best_last_index(self):
        #  print("get_best_last_index")
        YAWTH = np.deg2rad(3.0)
        XYTH = 0.5

        min_idx = None
        min_cost = np.inf
        for i, node in enumerate(self.nodeList):
            if self.calc_dist_to_goal(node.x, node.y) <= XYTH:
                if abs(node.yaw - self.end.yaw) <= YAWTH:
                    if node.cost < min_cost:
                        min_cost = node.cost
                        min_idx = i
        return min_idx

    def gen_final_course(self, goalind):
        path = [[self.end.x, self.end.y]]
        while self.nodeList[goalind].parent is not None:
            node = self.nodeList[goalind]
            for (ix, iy) in zip(reversed(node.path_x), reversed(node.path_y)):
                path.append([ix, iy])
            #  path.append([node.x, node.y])
            goalind = node.parent
        path.append([self.start.x, self.start.y])
        return path

    def calc_dist_to_goal(self, x, y):
        return np.linalg.norm([x - self.end.x, y - self.end.y])

    def find_near_nodes(self, newNode):
        nnode = len(self.nodeList)
        r = 50.0 * np.sqrt(np.log(nnode) / nnode)
        r_square = r ** 2
        nearinds = []
        for i, node in enumerate(self.nodeList):
            d = (node.x - newNode.x) ** 2 + (node.y - newNode.y) ** 2 + \
                (node.yaw - newNode.yaw) ** 2
            if d < r_square:
                nearinds.append(i)
        return nearinds

    def rewire(self, newNode, nearinds):
        nnode = len(self.nodeList)

        for i in nearinds:
            nearNode = self.nodeList[i]
            tNode = self.steer(nearNode, nnode - 1)
            if tNode is None:
                continue

            obstacleOK = self.CollisionCheck(tNode, self.obstacleList)
            imporveCost = nearNode.cost > tNode.cost

            if obstacleOK and imporveCost:
                #  print("rewire")
                self.nodeList[i] = tNode

    def DrawGraph(self, rnd=None):
        """
        Draw Graph
        """
        plt.clf()
        if rnd is not None:
            plt.plot(rnd.x, rnd.y, "^k")
        for node in self.nodeList:
            if node.parent is not None:
                plt.plot(node.path_x, node.path_y, "-g")
                #  plt.plot([node.x, self.nodeList[node.parent].x], [
                #  node.y, self.nodeList[node.parent].y], "-g")

        for (ox, oy, size) in self.obstacleList:
            plt.plot(ox, oy, "ok", ms=30 * size)

        reeds_shepp_path_planning.plot_arrow(
            self.start.x, self.start.y, self.start.yaw)
        reeds_shepp_path_planning.plot_arrow(
            self.end.x, self.end.y, self.end.yaw)

        plt.axis([-2, 15, -2, 15])
        plt.grid(True)
        plt.pause(0.01)

        #  plt.show()
        #  input()

    def GetNearestListIndex(self, nodeList, rnd):
        min_d = np.inf
        min_idx = 0
        for i, node in enumerate(nodeList):
            d = (node.x - rnd.x) ** 2 + (node.y - rnd.y) ** 2 + \
                (node.yaw - rnd.yaw) ** 2
            if d < min_d:
                min_d = d
                min_idx = i

        return min_idx

    def CollisionCheck(self, node, obstacleList):
        for (ox, oy, size) in obstacleList:
            for (ix, iy) in zip(node.path_x, node.path_y):
                dx = ox - ix
                dy = oy - iy
                d = dx * dx + dy * dy
                if d <= size ** 2:
                    return False  # collision

        return True  # safe


class Node():
    """
    RRT Node
    """

    def __init__(self, x, y, yaw):
        self.x = x
        self.y = y
        self.yaw = yaw
        self.path_x = []
        self.path_y = []
        self.path_yaw = []
        self.cost = 0.0
        self.parent = None


def main():
    print("Start rrt start planning")

    # ====Search Path with RRT====
    #  obstacleList = [
    #  (5, 5, 1),
    #  (3, 6, 2),
    #  (3, 8, 2),
    #  (3, 10, 2),
    #  (7, 5, 2),
    #  (9, 5, 2)
    #  ]  # [x,y,size(radius)]
    obstacleList = [
        (5, 5, 1),
        (4, 6, 1),
        (4, 8, 1),
        (4, 10, 1),
        (6, 5, 1),
        (7, 5, 1),
        (8, 6, 1),
        (8, 8, 1),
        (8, 10, 1)
    ]  # [x,y,size(radius)]

    # Set Initial parameters
    start = [0.0, 0.0, np.deg2rad(0.0)]
    goal = [6.0, 7.0, np.deg2rad(90.0)]

    rrt = RRT(start, goal, randArea=[-2.0, 15.0], obstacleList=obstacleList)
    path = rrt.Planning(animation=show_animation)

    # Draw final path
    if show_animation:
        rrt.DrawGraph()
        plt.plot([x for (x, y) in path], [y for (x, y) in path], '-r')
        plt.grid(True)
        plt.pause(0.001)
        plt.show()


if __name__ == '__main__':
    main()
