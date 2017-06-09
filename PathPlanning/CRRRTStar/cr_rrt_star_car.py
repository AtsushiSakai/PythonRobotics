#!/usr/bin/python
# -*- coding: utf-8 -*-
"""
@brief: Path Planning Sample Code with RRT for car like robot.

@author: AtsushiSakai(@Atsushi_twi)

@license: MIT

"""

import random
import math
import copy
import numpy as np
import dubins_path_planning
import pure_pursuit
import unicycle_model


class RRT():
    u"""
    Class for RRT Planning
    """

    def __init__(self, start, goal, obstacleList, randArea,
                 goalSampleRate=10, maxIter=100):
        u"""
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
        self.obstacleList = obstacleList
        self.maxIter = maxIter

    def Planning(self, animation=True):
        u"""
        Pathplanning

        animation: flag for animation on or off
        """

        self.nodeList = [self.start]
        for i in range(self.maxIter):
            rnd = self.get_random_point()
            nind = self.GetNearestListIndex(self.nodeList, rnd)

            newNode = self.steer(rnd, nind)
            #  print(newNode.cost)

            if self.CollisionCheck(newNode, obstacleList):
                nearinds = self.find_near_nodes(newNode)
                newNode = self.choose_parent(newNode, nearinds)
                self.nodeList.append(newNode)
                self.rewire(newNode, nearinds)

            if animation and i % 5 == 0:
                self.DrawGraph(rnd=rnd)
                matplotrecorder.save_frame()  # save each frame

        # generate coruse
        path_indexs = self.get_best_last_indexs()

        # pure pursuit tracking
        for ind in path_indexs:
            path = self.gen_final_course(ind)

            flag, x, y, yaw, v, t = self.check_tracking_path_is_feasible(path)

            if flag:
                print("feasible path is found")
                break

        return x, y, yaw, v, t

    def check_tracking_path_is_feasible(self, path):
        print("check_tracking_path_is_feasible")

        init_speed = 0.0
        target_speed = 10.0 / 3.6

        path = np.matrix(path[::-1])

        state = unicycle_model.State(
            x=self.start.x, y=self.start.y, yaw=self.start.yaw, v=init_speed)

        target_ind = pure_pursuit.calc_nearest_index(
            state, path[:, 0], path[:, 1])

        lastIndex = len(path[:, 0]) - 2

        x = [state.x]
        y = [state.y]
        yaw = [state.yaw]
        v = [state.v]
        t = [0.0]
        a = [0.0]
        d = [0.0]
        time = 0.0

        while lastIndex > target_ind:
            print(lastIndex, target_ind)
            ai = pure_pursuit.PIDControl(target_speed, state.v)
            di, target_ind = pure_pursuit.pure_pursuit_control(
                state, path[:, 0], path[:, 1], target_ind)
            state = unicycle_model.update(state, ai, di)

            time = time + unicycle_model.dt

            x.append(state.x)
            y.append(state.y)
            yaw.append(state.yaw)
            v.append(state.v)
            t.append(time)
            a.append(ai)
            d.append(di)

        if self.CollisionCheckWithXY(path[:, 0], path[:, 1], self.obstacleList):
            #  print("OK")
            return True, x, y, yaw, v, t, a, d
        else:
            #  print("NG")
            return False, x, y, yaw, v, t, a, d

        #  plt.plot(x, y, '-r')
        #  plt.plot(path[:, 0], path[:, 1], '-g')
        #  plt.show()

        #  return True

    def choose_parent(self, newNode, nearinds):
        if len(nearinds) == 0:
            return newNode

        dlist = []
        for i in nearinds:
            tNode = self.steer(newNode, i)
            if self.CollisionCheck(tNode, obstacleList):
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
        while(angle >= math.pi):
            angle = angle - 2.0 * math.pi

        while(angle <= -math.pi):
            angle = angle + 2.0 * math.pi

        return angle

    def steer(self, rnd, nind):
        #  print(rnd)
        curvature = 1.0

        nearestNode = self.nodeList[nind]

        px, py, pyaw, mode, clen = dubins_path_planning.dubins_path_planning(
            nearestNode.x, nearestNode.y, nearestNode.yaw, rnd.x, rnd.y, rnd.yaw, curvature)

        newNode = copy.deepcopy(nearestNode)
        newNode.x = px[-1]
        newNode.y = py[-1]
        newNode.yaw = pyaw[-1]

        newNode.path_x = px
        newNode.path_y = py
        newNode.path_yaw = pyaw
        newNode.cost += clen
        newNode.parent = nind

        return newNode

    def get_random_point(self):

        if random.randint(0, 100) > self.goalSampleRate:
            rnd = [random.uniform(self.minrand, self.maxrand),
                   random.uniform(self.minrand, self.maxrand),
                   random.uniform(-math.pi, math.pi)
                   ]
        else:  # goal point sampling
            rnd = [self.end.x, self.end.y, self.end.yaw]

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

        # angle check
        fgoalinds = []
        for i in goalinds:
            if abs(self.nodeList[i].yaw - self.end.yaw) <= YAWTH:
                fgoalinds.append(i)

        return fgoalinds

        #  mincost = min([self.nodeList[i].cost for i in fgoalinds])
        #  for i in fgoalinds:
        #  if self.nodeList[i].cost == mincost:
        #  return i

        #  return None

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

            obstacleOK = self.CollisionCheck(tNode, obstacleList)
            imporveCost = nearNode.cost > tNode.cost

            if obstacleOK and imporveCost:
                #  print("rewire")
                self.nodeList[i] = tNode

    def DrawGraph(self, rnd=None):
        u"""
        Draw Graph
        """
        import matplotlib.pyplot as plt
        #  plt.clf()
        if rnd is not None:
            plt.plot(rnd.x, rnd.y, "^k")
        for node in self.nodeList:
            if node.parent is not None:
                plt.plot(node.path_x, node.path_y, "-g")
                #  plt.plot([node.x, self.nodeList[node.parent].x], [
                #  node.y, self.nodeList[node.parent].y], "-g")

        for (ox, oy, size) in obstacleList:
            plt.plot(ox, oy, "ok", ms=30 * size)

        dubins_path_planning.plot_arrow(
            self.start.x, self.start.y, self.start.yaw)
        dubins_path_planning.plot_arrow(
            self.end.x, self.end.y, self.end.yaw)

        plt.axis([-2, 15, -2, 15])
        plt.grid(True)
        plt.pause(0.01)

        #  plt.show()
        #  input()

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
    u"""
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


if __name__ == '__main__':
    print("Start rrt start planning")
    import matplotlib.pyplot as plt
    import matplotrecorder
    matplotrecorder.donothing = True

    # ====Search Path with RRT====
    obstacleList = [
        (5, 5, 1),
        (3, 6, 2),
        (3, 8, 2),
        (3, 10, 2),
        (7, 5, 2),
        (9, 5, 2)
    ]  # [x,y,size(radius)]

    # Set Initial parameters
    start = [0.0, 0.0, math.radians(0.0)]
    goal = [10.0, 10.0, math.radians(0.0)]

    rrt = RRT(start, goal, randArea=[-2.0, 15.0], obstacleList=obstacleList)
    x, y, yaw, v, t = rrt.Planning(animation=False)

    flg, ax = plt.subplots(1)
    # Draw final path
    rrt.DrawGraph()
    plt.plot(x, y, '-r')
    plt.grid(True)
    plt.pause(0.001)

    for i in range(10):
        matplotrecorder.save_frame()  # save each frame

    flg, ax = plt.subplots(1)
    plt.plot(t, yaw, '-r')

    flg, ax = plt.subplots(1)
    plt.plot(t, v, '-r')

    plt.show()

    matplotrecorder.save_movie("animation.gif", 0.1)
