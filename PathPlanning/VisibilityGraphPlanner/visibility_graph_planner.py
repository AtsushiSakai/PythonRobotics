"""

Visibility Graph Planner

author: Atsushi Sakai (@Atsushi_twi)

"""

import os
import sys

sys.path.append(os.path.dirname(os.path.abspath(__file__)) +
                "/../VoronoiRoadMap/")

import math
import numpy as np
import matplotlib.pyplot as plt
from dijkstra_search import DijkstraSearch

show_animation = True


class VisibilityGraphPlanner:

    def __init__(self, robot_radius):
        self.robot_radius = robot_radius

    def planning(self, start_x, start_y, goal_x, goal_y, obstacles):
        nodes = self.extract_graph_node(start_x, start_y, goal_x, goal_y,
                                        obstacles)
        graph = self.generate_graph(nodes, obstacles)

        # rx, ry = DijkstraSearch().search(graph)

        rx, ry = [], []

        return rx, ry

    def extract_graph_node(self, start_x, start_y, goal_x, goal_y, obstacles):

        # add start and goal as nodes
        nodes = [DijkstraSearch.Node(start_x, start_y),
                 DijkstraSearch.Node(goal_x, goal_y, 0, None)]

        # add vertexes in configuration space as nodes
        for obstacle in obstacles:

            cvx_list, cvy_list = self.calc_vertexes_in_configuration_space(
                obstacle.x_list, obstacle.y_list)

            for (vx, vy) in zip(cvx_list, cvy_list):
                nodes.append(DijkstraSearch.Node(vx, vy))

        for node in nodes:
            plt.plot(node.x, node.y, "xr")

        return nodes

    def calc_vertexes_in_configuration_space(self, x_list, y_list):
        x_list=x_list[0:-1]
        y_list=y_list[0:-1]
        cvx_list, cvy_list = [], []

        n_data=len(x_list)

        for index in range(n_data):
            offset_x, offset_y = self.calc_offset_xy(
                x_list[index - 1], y_list[index - 1],
                x_list[index], y_list[index],
                x_list[(index + 1) % n_data], y_list[(index + 1) % n_data],
            )
            cvx_list.append(offset_x)
            cvy_list.append(offset_y)

        return cvx_list, cvy_list

    def generate_graph(self, nodes, obstacles):

        graph = []

        for target_node in nodes:
            for node in nodes:
                for obstacle in obstacles:
                    if not self.is_edge_valid(target_node, node, obstacle):
                        print("bb")
                        break
                    print(target_node, node)
                    print("aa")
                    plt.plot([target_node.x, node.x],[target_node.y, node.y], "-r")

        return graph

    def is_edge_valid(self, target_node, node, obstacle):

        for i in range(len(obstacle.x_list)-1):
            p1 = np.array([target_node.x, target_node.y])
            p2 = np.array([node.x, node.y])
            p3 = np.array([obstacle.x_list[i], obstacle.y_list[i]])
            p4 = np.array([obstacle.y_list[i+1], obstacle.y_list[i+1]])

            if is_seg_intersect(p1, p2, p3, p4):
                return False

        return True

    def calc_offset_xy(self, px, py, x, y, nx, ny):
        p_vec = math.atan2(y - py, x - px)
        n_vec = math.atan2(ny - y, nx - x)
        offset_vec = math.atan2(math.sin(p_vec) + math.sin(n_vec),
                                math.cos(p_vec) + math.cos(
                                    n_vec))+math.pi/2.0
        offset_x = x + self.robot_radius * math.cos(offset_vec)
        offset_y = y + self.robot_radius * math.sin(offset_vec)
        return offset_x, offset_y


def is_seg_intersect(a1, a2, b1, b2):

    xdiff = [a1[0] - a2[0], b1[0] - b2[0]]
    ydiff = [a1[1] - a2[1], b1[1] - b2[1]]

    def det(a, b):
        return a[0] * b[1] - a[1] * b[0]

    div = det(xdiff, ydiff)
    if div == 0:
        return False
    else:
        return True

class ObstaclePolygon:

    def __init__(self, x_list, y_list):
        self.x_list = x_list
        self.y_list = y_list

        self.close_polygon()
        self.make_clockwise()

    def make_clockwise(self):
        if not self.is_clockwise():
            self.x_list = list(reversed(self.x_list))
            self.y_list = list(reversed(self.y_list))

    def is_clockwise(self):
        n_data = len(self.x_list)
        eval_sum = sum([(self.x_list[i + 1] - self.x_list[i]) *
                        (self.y_list[i + 1] + self.y_list[i])
                        for i in range(n_data - 1)])
        eval_sum += (self.x_list[0] - self.x_list[n_data - 1]) * \
                    (self.y_list[0] + self.y_list[n_data - 1])
        return eval_sum >= 0

    def close_polygon(self):
        is_x_same = self.x_list[0] == self.x_list[-1]
        is_y_same = self.y_list[0] == self.y_list[-1]
        if is_x_same and is_y_same:
            return  # no need to close

        self.x_list.append(self.x_list[0])
        self.y_list.append(self.y_list[0])

    def plot(self):
        plt.plot(self.x_list, self.y_list, "-b")


def main():
    print(__file__ + " start!!")

    # start and goal position
    sx, sy = 10.0, 10.0  # [m]
    gx, gy = 50.0, 50.0  # [m]
    robot_radius = 5.0  # [m]

    obstacles = [ObstaclePolygon(
        [20.0, 30.0, 15.0],
        [20.0, 20.0, 30.0],
    ), ObstaclePolygon(
        [30.0, 45.0, 50.0, 40.0],
        [50.0, 40.0, 20.0, 40.0],
    )]

    if show_animation:  # pragma: no cover
        plt.plot(sx, sy, "or")
        plt.plot(gx, gy, "ob")
        [ob.plot() for ob in obstacles]

    rx, ry = VisibilityGraphPlanner(robot_radius).planning(sx, sy, gx, gy,
                                                           obstacles)
    # assert rx, 'Cannot found path'
    if show_animation:  # pragma: no cover
        plt.plot(rx, ry, "-r")
        plt.axis("equal")
        plt.show()


if __name__ == '__main__':
    main()
