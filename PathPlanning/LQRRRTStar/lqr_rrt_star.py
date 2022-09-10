"""

Path planning code with LQR RRT*

author: AtsushiSakai(@Atsushi_twi)

"""
import copy
import math
import random
import matplotlib.pyplot as plt
import numpy as np
import sys
import pathlib
sys.path.append(str(pathlib.Path(__file__).parent.parent))

from LQRPlanner.lqr_planner import LQRPlanner
from RRTStar.rrt_star import RRTStar

show_animation = True


class LQRRRTStar(RRTStar):
    """
    Class for RRT star planning with LQR planning
    """

    def __init__(self, start, goal, obstacle_list, rand_area,
                 goal_sample_rate=10,
                 max_iter=200,
                 connect_circle_dist=50.0,
                 step_size=0.2,
                 robot_radius=0.0,
                 ):
        """
        Setting Parameter

        start:Start Position [x,y]
        goal:Goal Position [x,y]
        obstacleList:obstacle Positions [[x,y,size],...]
        randArea:Random Sampling Area [min,max]
        robot_radius: robot body modeled as circle with given radius

        """
        self.start = self.Node(start[0], start[1])
        self.end = self.Node(goal[0], goal[1])
        self.min_rand = rand_area[0]
        self.max_rand = rand_area[1]
        self.goal_sample_rate = goal_sample_rate
        self.max_iter = max_iter
        self.obstacle_list = obstacle_list
        self.connect_circle_dist = connect_circle_dist

        self.curvature = 1.0
        self.goal_xy_th = 0.5
        self.step_size = step_size
        self.robot_radius = robot_radius

        self.lqr_planner = LQRPlanner()

    def planning(self, animation=True, search_until_max_iter=True):
        """
        RRT Star planning

        animation: flag for animation on or off
        """

        self.node_list = [self.start]
        for i in range(self.max_iter):
            print("Iter:", i, ", number of nodes:", len(self.node_list))
            rnd = self.get_random_node()
            nearest_ind = self.get_nearest_node_index(self.node_list, rnd)
            new_node = self.steer(self.node_list[nearest_ind], rnd)

            if self.check_collision(
                    new_node, self.obstacle_list, self.robot_radius):
                near_indexes = self.find_near_nodes(new_node)
                new_node = self.choose_parent(new_node, near_indexes)
                if new_node:
                    self.node_list.append(new_node)
                    self.rewire(new_node, near_indexes)

            if animation and i % 5 == 0:
                self.draw_graph(rnd)

            if (not search_until_max_iter) and new_node:  # check reaching the goal
                last_index = self.search_best_goal_node()
                if last_index:
                    return self.generate_final_course(last_index)

        print("reached max iteration")

        last_index = self.search_best_goal_node()
        if last_index:
            return self.generate_final_course(last_index)
        else:
            print("Cannot find path")

        return None

    def draw_graph(self, rnd=None):
        plt.clf()
        # for stopping simulation with the esc key.
        plt.gcf().canvas.mpl_connect('key_release_event',
                lambda event: [exit(0) if event.key == 'escape' else None])
        if rnd is not None:
            plt.plot(rnd.x, rnd.y, "^k")
        for node in self.node_list:
            if node.parent:
                plt.plot(node.path_x, node.path_y, "-g")

        for (ox, oy, size) in self.obstacle_list:
            plt.plot(ox, oy, "ok", ms=30 * size)

        plt.plot(self.start.x, self.start.y, "xr")
        plt.plot(self.end.x, self.end.y, "xr")
        plt.axis([-2, 15, -2, 15])
        plt.grid(True)
        plt.pause(0.01)

    def search_best_goal_node(self):
        dist_to_goal_list = [self.calc_dist_to_goal(n.x, n.y) for n in self.node_list]
        goal_inds = [dist_to_goal_list.index(i) for i in dist_to_goal_list if i <= self.goal_xy_th]

        if not goal_inds:
            return None

        min_cost = min([self.node_list[i].cost for i in goal_inds])
        for i in goal_inds:
            if self.node_list[i].cost == min_cost:
                return i

        return None

    def calc_new_cost(self, from_node, to_node):

        wx, wy = self.lqr_planner.lqr_planning(
            from_node.x, from_node.y, to_node.x, to_node.y, show_animation=False)

        px, py, course_lengths = self.sample_path(wx, wy, self.step_size)

        if not course_lengths:
            return float("inf")

        return from_node.cost + sum(course_lengths)

    def get_random_node(self):

        if random.randint(0, 100) > self.goal_sample_rate:
            rnd = self.Node(random.uniform(self.min_rand, self.max_rand),
                            random.uniform(self.min_rand, self.max_rand)
                            )
        else:  # goal point sampling
            rnd = self.Node(self.end.x, self.end.y)

        return rnd

    def generate_final_course(self, goal_index):
        print("final")
        path = [[self.end.x, self.end.y]]
        node = self.node_list[goal_index]
        while node.parent:
            for (ix, iy) in zip(reversed(node.path_x), reversed(node.path_y)):
                path.append([ix, iy])
            node = node.parent
        path.append([self.start.x, self.start.y])
        return path

    def sample_path(self, wx, wy, step):

        px, py, clen = [], [], []

        for i in range(len(wx) - 1):

            for t in np.arange(0.0, 1.0, step):
                px.append(t * wx[i + 1] + (1.0 - t) * wx[i])
                py.append(t * wy[i + 1] + (1.0 - t) * wy[i])

        dx = np.diff(px)
        dy = np.diff(py)

        clen = [math.sqrt(idx ** 2 + idy ** 2) for (idx, idy) in zip(dx, dy)]

        return px, py, clen

    def steer(self, from_node, to_node):

        wx, wy = self.lqr_planner.lqr_planning(
            from_node.x, from_node.y, to_node.x, to_node.y, show_animation=False)

        px, py, course_lens = self.sample_path(wx, wy, self.step_size)

        if px is None:
            return None

        newNode = copy.deepcopy(from_node)
        newNode.x = px[-1]
        newNode.y = py[-1]
        newNode.path_x = px
        newNode.path_y = py
        newNode.cost += sum([abs(c) for c in course_lens])
        newNode.parent = from_node

        return newNode


def main(maxIter=200):
    print("Start " + __file__)

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

    lqr_rrt_star = LQRRRTStar(start, goal,
                              obstacleList,
                              [-2.0, 15.0])
    path = lqr_rrt_star.planning(animation=show_animation)

    # Draw final path
    if show_animation:  # pragma: no cover
        lqr_rrt_star.draw_graph()
        plt.plot([x for (x, y) in path], [y for (x, y) in path], '-r')
        plt.grid(True)
        plt.pause(0.001)
        plt.show()

    print("Done")


if __name__ == '__main__':
    main()
