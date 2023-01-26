"""
Informed RRT* path planning

author: Karan Chawla
        Atsushi Sakai(@Atsushi_twi)

Reference: Informed RRT*: Optimal Sampling-based Path planning Focused via
Direct Sampling of an Admissible Ellipsoidal Heuristic
https://arxiv.org/pdf/1404.2334.pdf

"""
import sys
import pathlib

sys.path.append(str(pathlib.Path(__file__).parent.parent.parent))

import copy
import math
import random

import matplotlib.pyplot as plt
import numpy as np

from utils.angle import rot_mat_2d

show_animation = True


class InformedRRTStar:

    def __init__(self, start, goal, obstacle_list, rand_area, expand_dis=0.5,
                 goal_sample_rate=10, max_iter=200):

        self.start = Node(start[0], start[1])
        self.goal = Node(goal[0], goal[1])
        self.min_rand = rand_area[0]
        self.max_rand = rand_area[1]
        self.expand_dis = expand_dis
        self.goal_sample_rate = goal_sample_rate
        self.max_iter = max_iter
        self.obstacle_list = obstacle_list
        self.node_list = None

    def informed_rrt_star_search(self, animation=True):

        self.node_list = [self.start]
        # max length we expect to find in our 'informed' sample space,
        # starts as infinite
        c_best = float('inf')
        solution_set = set()
        path = None

        # Computing the sampling space
        c_min = math.hypot(self.start.x - self.goal.x,
                           self.start.y - self.goal.y)
        x_center = np.array([[(self.start.x + self.goal.x) / 2.0],
                             [(self.start.y + self.goal.y) / 2.0], [0]])
        a1 = np.array([[(self.goal.x - self.start.x) / c_min],
                       [(self.goal.y - self.start.y) / c_min], [0]])

        e_theta = math.atan2(a1[1], a1[0])
        # first column of identity matrix transposed
        id1_t = np.array([1.0, 0.0, 0.0]).reshape(1, 3)
        m = a1 @ id1_t
        u, s, vh = np.linalg.svd(m, True, True)
        c = u @ np.diag(
            [1.0, 1.0,
             np.linalg.det(u) * np.linalg.det(np.transpose(vh))]) @ vh

        for i in range(self.max_iter):
            # Sample space is defined by c_best
            # c_min is the minimum distance between the start point and
            # the goal x_center is the midpoint between the start and the
            # goal c_best changes when a new path is found

            rnd = self.informed_sample(c_best, c_min, x_center, c)
            n_ind = self.get_nearest_list_index(self.node_list, rnd)
            nearest_node = self.node_list[n_ind]
            # steer
            theta = math.atan2(rnd[1] - nearest_node.y,
                               rnd[0] - nearest_node.x)
            new_node = self.get_new_node(theta, n_ind, nearest_node)
            d = self.line_cost(nearest_node, new_node)

            no_collision = self.check_collision(nearest_node, theta, d)

            if no_collision:
                near_inds = self.find_near_nodes(new_node)
                new_node = self.choose_parent(new_node, near_inds)

                self.node_list.append(new_node)
                self.rewire(new_node, near_inds)

                if self.is_near_goal(new_node):
                    if self.check_segment_collision(new_node.x, new_node.y,
                                                    self.goal.x, self.goal.y):
                        solution_set.add(new_node)
                        last_index = len(self.node_list) - 1
                        temp_path = self.get_final_course(last_index)
                        temp_path_len = self.get_path_len(temp_path)
                        if temp_path_len < c_best:
                            path = temp_path
                            c_best = temp_path_len
            if animation:
                self.draw_graph(x_center=x_center, c_best=c_best, c_min=c_min,
                                e_theta=e_theta, rnd=rnd)

        return path

    def choose_parent(self, new_node, near_inds):
        if len(near_inds) == 0:
            return new_node

        d_list = []
        for i in near_inds:
            dx = new_node.x - self.node_list[i].x
            dy = new_node.y - self.node_list[i].y
            d = math.hypot(dx, dy)
            theta = math.atan2(dy, dx)
            if self.check_collision(self.node_list[i], theta, d):
                d_list.append(self.node_list[i].cost + d)
            else:
                d_list.append(float('inf'))

        min_cost = min(d_list)
        min_ind = near_inds[d_list.index(min_cost)]

        if min_cost == float('inf'):
            print("min cost is inf")
            return new_node

        new_node.cost = min_cost
        new_node.parent = min_ind

        return new_node

    def find_near_nodes(self, new_node):
        n_node = len(self.node_list)
        r = 50.0 * math.sqrt((math.log(n_node) / n_node))
        d_list = [(node.x - new_node.x) ** 2 + (node.y - new_node.y) ** 2 for
                  node in self.node_list]
        near_inds = [d_list.index(i) for i in d_list if i <= r ** 2]
        return near_inds

    def informed_sample(self, c_max, c_min, x_center, c):
        if c_max < float('inf'):
            r = [c_max / 2.0, math.sqrt(c_max ** 2 - c_min ** 2) / 2.0,
                 math.sqrt(c_max ** 2 - c_min ** 2) / 2.0]
            rl = np.diag(r)
            x_ball = self.sample_unit_ball()
            rnd = np.dot(np.dot(c, rl), x_ball) + x_center
            rnd = [rnd[(0, 0)], rnd[(1, 0)]]
        else:
            rnd = self.sample_free_space()

        return rnd

    @staticmethod
    def sample_unit_ball():
        a = random.random()
        b = random.random()

        if b < a:
            a, b = b, a

        sample = (b * math.cos(2 * math.pi * a / b),
                  b * math.sin(2 * math.pi * a / b))
        return np.array([[sample[0]], [sample[1]], [0]])

    def sample_free_space(self):
        if random.randint(0, 100) > self.goal_sample_rate:
            rnd = [random.uniform(self.min_rand, self.max_rand),
                   random.uniform(self.min_rand, self.max_rand)]
        else:
            rnd = [self.goal.x, self.goal.y]

        return rnd

    @staticmethod
    def get_path_len(path):
        path_len = 0
        for i in range(1, len(path)):
            node1_x = path[i][0]
            node1_y = path[i][1]
            node2_x = path[i - 1][0]
            node2_y = path[i - 1][1]
            path_len += math.hypot(node1_x - node2_x, node1_y - node2_y)

        return path_len

    @staticmethod
    def line_cost(node1, node2):
        return math.hypot(node1.x - node2.x, node1.y - node2.y)

    @staticmethod
    def get_nearest_list_index(nodes, rnd):
        d_list = [(node.x - rnd[0]) ** 2 + (node.y - rnd[1]) ** 2 for node in
                  nodes]
        min_index = d_list.index(min(d_list))
        return min_index

    def get_new_node(self, theta, n_ind, nearest_node):
        new_node = copy.deepcopy(nearest_node)

        new_node.x += self.expand_dis * math.cos(theta)
        new_node.y += self.expand_dis * math.sin(theta)

        new_node.cost += self.expand_dis
        new_node.parent = n_ind
        return new_node

    def is_near_goal(self, node):
        d = self.line_cost(node, self.goal)
        if d < self.expand_dis:
            return True
        return False

    def rewire(self, new_node, near_inds):
        n_node = len(self.node_list)
        for i in near_inds:
            near_node = self.node_list[i]

            d = math.hypot(near_node.x - new_node.x, near_node.y - new_node.y)

            s_cost = new_node.cost + d

            if near_node.cost > s_cost:
                theta = math.atan2(new_node.y - near_node.y,
                                   new_node.x - near_node.x)
                if self.check_collision(near_node, theta, d):
                    near_node.parent = n_node - 1
                    near_node.cost = s_cost

    @staticmethod
    def distance_squared_point_to_segment(v, w, p):
        # Return minimum distance between line segment vw and point p
        if np.array_equal(v, w):
            return (p - v).dot(p - v)  # v == w case
        l2 = (w - v).dot(w - v)  # i.e. |w-v|^2 -  avoid a sqrt
        # Consider the line extending the segment,
        # parameterized as v + t (w - v).
        # We find projection of point p onto the line.
        # It falls where t = [(p-v) . (w-v)] / |w-v|^2
        # We clamp t from [0,1] to handle points outside the segment vw.
        t = max(0, min(1, (p - v).dot(w - v) / l2))
        projection = v + t * (w - v)  # Projection falls on the segment
        return (p - projection).dot(p - projection)

    def check_segment_collision(self, x1, y1, x2, y2):
        for (ox, oy, size) in self.obstacle_list:
            dd = self.distance_squared_point_to_segment(
                np.array([x1, y1]), np.array([x2, y2]), np.array([ox, oy]))
            if dd <= size ** 2:
                return False  # collision
        return True

    def check_collision(self, near_node, theta, d):
        tmp_node = copy.deepcopy(near_node)
        end_x = tmp_node.x + math.cos(theta) * d
        end_y = tmp_node.y + math.sin(theta) * d
        return self.check_segment_collision(tmp_node.x, tmp_node.y,
                                            end_x, end_y)

    def get_final_course(self, last_index):
        path = [[self.goal.x, self.goal.y]]
        while self.node_list[last_index].parent is not None:
            node = self.node_list[last_index]
            path.append([node.x, node.y])
            last_index = node.parent
        path.append([self.start.x, self.start.y])
        return path

    def draw_graph(self, x_center=None, c_best=None, c_min=None, e_theta=None,
                   rnd=None):
        plt.clf()
        # for stopping simulation with the esc key.
        plt.gcf().canvas.mpl_connect(
            'key_release_event', lambda event:
            [exit(0) if event.key == 'escape' else None])
        if rnd is not None:
            plt.plot(rnd[0], rnd[1], "^k")
            if c_best != float('inf'):
                self.plot_ellipse(x_center, c_best, c_min, e_theta)

        for node in self.node_list:
            if node.parent is not None:
                if node.x or node.y is not None:
                    plt.plot([node.x, self.node_list[node.parent].x],
                             [node.y, self.node_list[node.parent].y], "-g")

        for (ox, oy, size) in self.obstacle_list:
            plt.plot(ox, oy, "ok", ms=30 * size)

        plt.plot(self.start.x, self.start.y, "xr")
        plt.plot(self.goal.x, self.goal.y, "xr")
        plt.axis([-2, 15, -2, 15])
        plt.grid(True)
        plt.pause(0.01)

    @staticmethod
    def plot_ellipse(x_center, c_best, c_min, e_theta):  # pragma: no cover

        a = math.sqrt(c_best ** 2 - c_min ** 2) / 2.0
        b = c_best / 2.0
        angle = math.pi / 2.0 - e_theta
        cx = x_center[0]
        cy = x_center[1]
        t = np.arange(0, 2 * math.pi + 0.1, 0.1)
        x = [a * math.cos(it) for it in t]
        y = [b * math.sin(it) for it in t]
        fx = rot_mat_2d(-angle) @ np.array([x, y])
        px = np.array(fx[0, :] + cx).flatten()
        py = np.array(fx[1, :] + cy).flatten()
        plt.plot(cx, cy, "xc")
        plt.plot(px, py, "--c")


class Node:

    def __init__(self, x, y):
        self.x = x
        self.y = y
        self.cost = 0.0
        self.parent = None


def main():
    print("Start informed rrt star planning")

    # create obstacles
    obstacle_list = [(5, 5, 0.5), (9, 6, 1), (7, 5, 1), (1, 5, 1), (3, 6, 1),
                     (7, 9, 1)]

    # Set params
    rrt = InformedRRTStar(start=[0, 0], goal=[5, 10], rand_area=[-2, 15],
                          obstacle_list=obstacle_list)
    path = rrt.informed_rrt_star_search(animation=show_animation)
    print("Done!!")

    # Plot path
    if show_animation:
        rrt.draw_graph()
        plt.plot([x for (x, y) in path], [y for (x, y) in path], '-r')
        plt.grid(True)
        plt.pause(0.01)
        plt.show()


if __name__ == '__main__':
    main()
