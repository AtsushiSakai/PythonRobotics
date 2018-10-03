"""
Path Planning Sample Code with Closed loop RRT for car like robot.

author: AtsushiSakai(@Atsushi_twi)

"""

import sys
sys.path.append("../ReedsSheppPath/")

import random
import math
import copy
import numpy as np
import pure_pursuit
import matplotlib.pyplot as plt

import reeds_shepp_path_planning
import unicycle_model

show_animation = True


target_speed = 10.0 / 3.6
STEP_SIZE = 0.1


class RRT():
    """
    Class for RRT Planning
    """

    def __init__(self, start, goal, obstacleList, randArea,
                 maxIter=200):
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
        self.obstacleList = obstacleList
        self.maxIter = maxIter

    def try_goal_path(self):

        goal = Node(self.end.x, self.end.y, self.end.yaw)

        newNode = self.steer(goal, len(self.nodeList) - 1)
        if newNode is None:
            return

        if self.CollisionCheck(newNode, self.obstacleList):
            #  print("goal path is OK")
            self.nodeList.append(newNode)

    def Planning(self, animation=True):
        """
        Pathplanning

        animation: flag for animation on or off
        """

        self.nodeList = [self.start]

        self.try_goal_path()

        for i in range(self.maxIter):
            rnd = self.get_random_point()
            nind = self.GetNearestListIndex(self.nodeList, rnd)

            newNode = self.steer(rnd, nind)
            #  print(newNode.cost)
            if newNode is None:
                continue

            if self.CollisionCheck(newNode, self.obstacleList):
                nearinds = self.find_near_nodes(newNode)
                newNode = self.choose_parent(newNode, nearinds)
                if newNode is None:
                    continue

                self.nodeList.append(newNode)
                self.rewire(newNode, nearinds)

                self.try_goal_path()

            if animation and i % 5 == 0:
                self.DrawGraph(rnd=rnd)

        # generate coruse
        path_indexs = self.get_best_last_indexs()

        flag, x, y, yaw, v, t, a, d = self.search_best_feasible_path(
            path_indexs)

        return flag, x, y, yaw, v, t, a, d

    def search_best_feasible_path(self, path_indexs):

        print("Start search feasible path")

        best_time = float("inf")

        fx = None

        # pure pursuit tracking
        for ind in path_indexs:
            path = self.gen_final_course(ind)

            flag, x, y, yaw, v, t, a, d = self.check_tracking_path_is_feasible(
                path)

            if flag and best_time >= t[-1]:
                print("feasible path is found")
                best_time = t[-1]
                fx, fy, fyaw, fv, ft, fa, fd = x, y, yaw, v, t, a, d

        print("best time is")
        print(best_time)

        if fx:
            fx.append(self.end.x)
            fy.append(self.end.y)
            fyaw.append(self.end.yaw)
            return True, fx, fy, fyaw, fv, ft, fa, fd
        else:
            return False, None, None, None, None, None, None, None

    def calc_tracking_path(self, path):
        path = np.matrix(path[::-1])
        ds = 0.2
        for i in range(10):
            lx = path[-1, 0]
            ly = path[-1, 1]
            lyaw = path[-1, 2]
            move_yaw = math.atan2(path[-2, 1] - ly, path[-2, 0] - lx)
            if abs(lyaw - move_yaw) >= math.pi / 2.0:
                print("back")
                ds *= -1

            lstate = np.matrix(
                [lx + ds * math.cos(lyaw), ly + ds * math.sin(lyaw), lyaw])
            #  print(lstate)

            path = np.vstack((path, lstate))

        return path

    def check_tracking_path_is_feasible(self, path):
        #  print("check_tracking_path_is_feasible")
        cx = np.array(path[:, 0])
        cy = np.array(path[:, 1])
        cyaw = np.array(path[:, 2])

        goal = [cx[-1], cy[-1], cyaw[-1]]

        cx, cy, cyaw = pure_pursuit.extend_path(cx, cy, cyaw)

        speed_profile = pure_pursuit.calc_speed_profile(
            cx, cy, cyaw, target_speed)

        t, x, y, yaw, v, a, d, find_goal = pure_pursuit.closed_loop_prediction(
            cx, cy, cyaw, speed_profile, goal)
        yaw = [self.pi_2_pi(iyaw) for iyaw in yaw]

        if not find_goal:
            print("cannot reach goal")

        if abs(yaw[-1] - goal[2]) >= math.pi / 4.0:
            print("final angle is bad")
            find_goal = False

        travel = sum([abs(iv) * unicycle_model.dt for iv in v])
        #  print(travel)
        origin_travel = sum([math.sqrt(dx ** 2 + dy ** 2)
                             for (dx, dy) in zip(np.diff(cx), np.diff(cy))])
        #  print(origin_travel)

        if (travel / origin_travel) >= 5.0:
            print("path is too long")
            find_goal = False

        if not self.CollisionCheckWithXY(x, y, self.obstacleList):
            print("This path is collision")
            find_goal = False

        return find_goal, x, y, yaw, v, t, a, d

    def choose_parent(self, newNode, nearinds):
        if len(nearinds) == 0:
            return newNode

        dlist = []
        for i in nearinds:
            tNode = self.steer(newNode, i)
            if tNode is None:
                continue

            if self.CollisionCheck(tNode, self.obstacleList):
                dlist.append(tNode.cost)
            else:
                dlist.append(float("inf"))

        mincost = min(dlist)
        minind = nearinds[dlist.index(mincost)]

        if mincost == float("inf"):
            print("mincost is inf")
            return newNode

        newNode = self.steer(newNode, minind)
        if newNode is None:
            return None

        return newNode


    def pi_2_pi(self, angle):
        return (angle + math.pi) % (2*math.pi) - math.pi


    def steer(self, rnd, nind):
        #  print(rnd)

        nearestNode = self.nodeList[nind]

        px, py, pyaw, mode, clen = reeds_shepp_path_planning.reeds_shepp_path_planning(
            nearestNode.x, nearestNode.y, nearestNode.yaw,
            rnd.x, rnd.y, rnd.yaw, unicycle_model.curvature_max, STEP_SIZE)

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

        rnd = [random.uniform(self.minrand, self.maxrand),
               random.uniform(self.minrand, self.maxrand),
               random.uniform(-math.pi, math.pi)
               ]

        node = Node(rnd[0], rnd[1], rnd[2])

        return node

    def get_best_last_indexs(self):
        #  print("get_best_last_index")

        YAWTH = math.radians(1.0)
        XYTH = 0.5

        goalinds = []
        for (i, node) in enumerate(self.nodeList):
            if self.calc_dist_to_goal(node.x, node.y) <= XYTH:
                goalinds.append(i)
        print("OK XY TH num is")
        print(len(goalinds))

        # angle check
        fgoalinds = []
        for i in goalinds:
            if abs(self.nodeList[i].yaw - self.end.yaw) <= YAWTH:
                fgoalinds.append(i)
        print("OK YAW TH num is")
        print(len(fgoalinds))

        return fgoalinds

    def gen_final_course(self, goalind):
        path = [[self.end.x, self.end.y, self.end.yaw]]
        while self.nodeList[goalind].parent is not None:
            node = self.nodeList[goalind]
            path_x = reversed(node.path_x)
            path_y = reversed(node.path_y)
            path_yaw = reversed(node.path_yaw)
            for (ix, iy, iyaw) in zip(path_x, path_y, path_yaw):
                path.append([ix, iy, iyaw])
            #  path.append([node.x, node.y])
            goalind = node.parent
        path.append([self.start.x, self.start.y, self.start.yaw])

        path = np.matrix(path[::-1])
        return path

    def calc_dist_to_goal(self, x, y):
        return np.linalg.norm([x - self.end.x, y - self.end.y])

    def find_near_nodes(self, newNode):
        nnode = len(self.nodeList)
        r = 50.0 * math.sqrt((math.log(nnode) / nnode))
        #  r = self.expandDis * 5.0
        dlist = [(node.x - newNode.x) ** 2 +
                 (node.y - newNode.y) ** 2 +
                 (node.yaw - newNode.yaw) ** 2
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

            obstacleOK = self.CollisionCheck(tNode, self.obstacleList)
            imporveCost = nearNode.cost > tNode.cost

            if obstacleOK and imporveCost:
                #  print("rewire")
                self.nodeList[i] = tNode

    def DrawGraph(self, rnd=None):
        """
        Draw Graph
        """
        if rnd is not None:
            plt.plot(rnd.x, rnd.y, "^k")
        for node in self.nodeList:
            if node.parent is not None:
                plt.plot(node.path_x, node.path_y, "-g")

        for (ox, oy, size) in self.obstacleList:
            plt.plot(ox, oy, "ok", ms=30 * size)

        reeds_shepp_path_planning.plot_arrow(
            self.start.x, self.start.y, self.start.yaw)
        reeds_shepp_path_planning.plot_arrow(
            self.end.x, self.end.y, self.end.yaw)

        plt.axis([-2, 15, -2, 15])
        plt.grid(True)
        plt.pause(0.01)

    def GetNearestListIndex(self, nodeList, rnd):
        dlist = [(node.x - rnd.x) ** 2 +
                 (node.y - rnd.y) ** 2 +
                 (node.yaw - rnd.yaw) ** 2 for node in nodeList]
        minind = dlist.index(min(dlist))

        return minind

    def CollisionCheck(self, node, obstacleList):

        for (ox, oy, size) in obstacleList:
            for (ix, iy) in zip(node.path_x, node.path_y):
                dx = ox - ix
                dy = oy - iy
                d = dx * dx + dy * dy
                if d <= size ** 2:
                    return False  # collision

        return True  # safe

    def CollisionCheckWithXY(self, x, y, obstacleList):

        for (ox, oy, size) in obstacleList:
            for (ix, iy) in zip(x, y):
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
    start = [0.0, 0.0, math.radians(0.0)]
    goal = [6.0, 7.0, math.radians(90.0)]

    rrt = RRT(start, goal, randArea=[-2.0, 20.0], obstacleList=obstacleList)
    flag, x, y, yaw, v, t, a, d = rrt.Planning(animation=show_animation)

    if not flag:
        print("cannot find feasible path")

    #  flg, ax = plt.subplots(1)
    # Draw final path
    if show_animation:
        rrt.DrawGraph()
        plt.plot(x, y, '-r')
        plt.grid(True)
        plt.pause(0.001)

        flg, ax = plt.subplots(1)
        plt.plot(t, [math.degrees(iyaw) for iyaw in yaw[:-1]], '-r')
        plt.xlabel("time[s]")
        plt.ylabel("Yaw[deg]")
        plt.grid(True)

        flg, ax = plt.subplots(1)
        plt.plot(t, [iv * 3.6 for iv in v], '-r')

        plt.xlabel("time[s]")
        plt.ylabel("velocity[km/h]")
        plt.grid(True)

        flg, ax = plt.subplots(1)
        plt.plot(t, a, '-r')
        plt.xlabel("time[s]")
        plt.ylabel("accel[m/ss]")
        plt.grid(True)

        flg, ax = plt.subplots(1)
        plt.plot(t, [math.degrees(td) for td in d], '-r')
        plt.xlabel("time[s]")
        plt.ylabel("Steering angle[deg]")
        plt.grid(True)

        plt.show()


if __name__ == '__main__':
    main()
