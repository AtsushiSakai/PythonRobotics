"""
D* Lite grid planning
author: vss2sn (28676655+vss2sn@users.noreply.github.com)
Link to papers:
D* Lite (http://idm-lab.org/bib/abstracts/papers/aaai02b.pd)
Improved Fast Replanning for Robot Navigation in Unknown Terrain (http://www.cs.cmu.edu/~maxim/files/dlite_icra02.pdf)
Implemented maintaining similarity with the pseudocode for understanding
"""

import math
import matplotlib.pyplot as plt
import random

show_animation = True
pause_time = 0.1

class Node:

    def __init__(self, x: int=0, y: int=0, cost: double=0.0):
        self.x = x
        self.y = y
        self.cost = cost

    @staticmethod
    def add_coordinates(node1: Node, node2: Node):
        new_node = Node()
        new_node.x = node1.x + node2.x
        new_node.y = node1.y + node2.y
        new_node.cost =  node1.cost + node2.cost
        return new_node

    @staticmethod
    def compare_coordinates(node1: Node, node2: Node):
        return node1.x == node2.x and node1.y == node2.y

class DStartLite:

    motions = [
        Node(1,0,1),
        Node(0,1,1),
        Node(-1,0,1),
        Node(0,-1,1)
    ]

    def __init__(self, x_max: int, y_max: int, obstacles:list, spoofed_obstacles:list=list()):
        self.x_max = x_max
        self.y_max = y_max
        self.obstacles = obstacles
        self.spoofed_obstacles = spoofed_obstacles
        self.start = Node(0,0)
        self.goal = Node(0,0)
        self.U = list() # Must be ordered; meets guidelines as os guaranteed ordered from 3.7+
        self.km = 0
        self.kold = 0
        self.rhs = list()
        self.g = list()
        self.detected_obstacles = list()
        if show_animation:
            self.detected_obstacles_for_plotting_x = list()
            self.detected_obstacles_for_plotting_y = list()

    def create_grid(self, val: double):
        grid = list()
        for _ in range(0, self.x_max):
            grid_row = list()
            for _ in range(0, self.y_max):
                grid_row.append(val)
            grid.append(grid_row)
        return grid

    def is_obstacle(self, node: Node):
        return any([Node.compare_coordinates(node, obstacle) for obstacle in self.obstacles]) or \
               any([Node.compare_coordinates(node, obstacle) for obstacle in self.detected_obstacles])

    def c(self, node1: Node, node2: Node):
        if self.is_obstacle(node2): # Attempting to move to an obstacle
            return math.inf
        new_node = Node(node1.x-node2.x, node1.y-node2.y)
        detected_motion = list(filter(lambda motion: Node.compare_coordinates(motion, new_node), self.motions))
        return detected_motion[0].cost

    def h(self, s: Node):
        return abs(self.start.x - s.x) + abs(self.start.y - s.y) # Modify if allowing diagonal motion

    def calculate_key(self, s: Node):
        return min(self.g[s.x][s.y], self.rhs[s.x][s.y]) + self.h(s) + self.km,  min(self.g[s.x][s.y] , self.rhs[s.x][s.y])

    def is_valid(self, node: Node):
        if node.x >= 0 and node.y >= 0 and node.x < self.x_max and node.y < self.x_max and not self.is_obstacle(node):
            return True
        return False

    def get_neighbours(self, u: Node):
        return [Node.add_coordinates(u, motion) for motion in self.motions if self.is_valid(Node.add_coordinates(u, motion))]

    def pred(self, u: Node):
        return self.get_neighbours(u)  # Grid, so each vertex is connected to the ones around it

    def succ(self, u: Node):
        return self.get_neighbours(u)  # Grid, so each vertex is connected to the ones around it

    def initialize(self, start: Node, goal: Node):
        self.start = start
        self.goal = goal
        self.U = list()
        self.km = 0
        self.rhs = self.create_grid(math.inf)
        self.g = self.create_grid(math.inf)
        self.rhs[goal.x][goal.y] = 0
        self.U.append((goal, self.calculate_key(goal)))
        self.detected_obstacles = list()

    def update_vertex(self, u: Node):
        if not Node.compare_coordinates(u, self.goal):
            self.rhs[u.x][u.y] = min([self.c(u, sprime) + self.g[sprime.x][sprime.y] for sprime in self.succ(u)])
        if any([Node.compare_coordinates(u, node) for node, key in self.U]):
            self.U = [(node, key) for node, key in self.U if not Node.compare_coordinates(node, u)]
            self.U.sort(key=lambda x: x[1])
        if self.g[u.x][u.y] != self.rhs[u.x][u.y]:
            self.U.append((u, self.calculate_key(u)))
            self.U.sort(key=lambda x: x[1])

    def compare_keys(self, key_pair1: tuple[double, double], key_pair2: tuple[double, double]):
        return key_pair1[0] < key_pair2[0] or (key_pair1[0] == key_pair2[0] and key_pair1[1] < key_pair2[1])

    def compute_shortest_path(self):
        self.U.sort(key=lambda x: x[1])
        while (len(self.U) > 0 and self.compare_keys(self.U[0][1], self.calculate_key(self.start))) or self.rhs[self.start.x][self.start.y] != self.g[self.start.x][self.start.y]:
            u = self.U[0][0]
            self.kold = self.U[0][1]
            self.U.pop(0)
            if self.compare_keys(self.kold, self.calculate_key(u)):
                self.U.append((u, self.calculate_key(u)))
                self.U.sort(key=lambda x: x[1])
            elif self.g[u.x][u.y] > self.rhs[u.x][u.y]:
                self.g[u.x][u.y] = self.rhs[u.x][u.y]
                for s in self.pred(u):
                    self.update_vertex(s)
            else:
                self.g[u.x][u.y] = math.inf
                for s in self.pred(u) + [u]:
                    self.update_vertex(s)


    def detect_changes(self):
        changed_vertices = list()
        if len(self.spoofed_obstacles) > 0:
            for spoofed_obstacle in self.spoofed_obstacles[0]:
                if Node.compare_coordinates(spoofed_obstacle, self.start) or \
                   Node.compare_coordinates(spoofed_obstacle, self.goal):
                    continue
                changed_vertices.append(spoofed_obstacle)
                self.detected_obstacles.append(spoofed_obstacle)
                if show_animation:
                    self.detected_obstacles_for_plotting_x.append(spoofed_obstacle.x)
                    self.detected_obstacles_for_plotting_y.append(spoofed_obstacle.y)
                    plt.plot(self.detected_obstacles_for_plotting_x, self.detected_obstacles_for_plotting_y, ".k")
                    plt.pause(pause_time)
            self.spoofed_obstacles.pop(0)
        # Allows random generation of obstacles
        # random.seed(1)
        # p_generate_obs = 0.25; #probability of randomly generating an obstacle
        # if random.random() > 1 - p_generate_obs:
        #     x = random.randint(0, self.x_max)
        #     y = random.randint(0, self.y_max)
        #     changed_vertices.append(Node(x, y))
        #     self.detected_obstacles.append(Node(x, y))
        #     if show_animation:
        #         self.detected_obstacles_for_plotting_x.append(x)
        #         self.detected_obstacles_for_plotting_y.append(y)
        #         plt.plot(self.detected_obstacles_for_plotting_x, self.detected_obstacles_for_plotting_y, ".k")
        #         plt.pause(0.01)
        return changed_vertices

    def main(self, start: Node, goal: Node):
        pathx = []
        pathy = []
        last = start
        self.initialize(start, goal)
        self.compute_shortest_path()
        pathx.append(start.x)
        pathy.append(start.y)
        while not Node.compare_coordinates(self.goal, self.start):
            if self.g[start.x][start.y] == math.inf:
                print("No path possible")
                return
            self.start = min([sprime for sprime in self.succ(self.start)], key=lambda sprime: self.c(self.start, sprime) + self.g[sprime.x][sprime.y])
            pathx.append(self.start.x)
            pathy.append(self.start.y)
            if show_animation:
                plt.plot(pathx, pathy, "-r")
                plt.axis("equal")
                plt.pause(pause_time)
            changed_vertices = self.detect_changes()
            if changed_vertices != None:
                self.km += self.h(last, self.start)
                last = self.start
                for u in changed_vertices:
                    if Node.compare_coordinates(u, self.start):
                        continue
                    # Not required, can use for debug
                    # self.rhs[u.x][u.y] = math.inf
                    # self.g[u.x][u.y] = math.inf
                    for neighbour in self.get_neighbours(u):
                        self.update_vertex(neighbour)
                self.compute_shortest_path()
        return pathx, pathy


def main():
    start = [0, 0]
    goal = [5, 5]

    obsx = [1, 3]
    obsy = [1, 2]
    obstacles = [Node(x, y) for x,y in zip(obsx, obsy)]

    # Obstacles discovered at time = row
    # time = 1, obstacles discovered at (0, 2), (9, 2), (4, 0)
    # time = 2, obstacles discovered at (0, 1), (7, 6)
    # ...
    spoofed_obstacles = list()
    spoofed_obstacles_x = [[0, 9, 4], [0, 7], [], [], [], [], [], [5]]
    spoofed_obstacles_y = [[2, 2, 0], [1, 6], [], [], [], [], [], [4]]
    for rowx, rowy in zip(spoofed_obstacles_x, spoofed_obstacles_y):
        spoofed_obstacles_row = [Node(x, y) for x,y in zip(rowx, rowy)]
        spoofed_obstacles.append(spoofed_obstacles_row)

    start_node = Node(x=start[0], y=start[1])
    goal_node = Node(x=goal[0], y=goal[1])

    if show_animation:
        plt.plot(obsx, obsy, ".k")
        plt.plot(start[0], start[1], "og")
        plt.plot(goal[0], goal[1], "xb")
        plt.axis("equal")

    dstarlite = DStartLite(x_max=10, y_max=10, obstacles=obstacles, spoofed_obstacles=spoofed_obstacles)
    pathx, pathy = dstarlite.main(start_node, goal_node)

    if show_animation:
        plt.plot(pathx, pathy, "-r")
        plt.axis("equal")
        plt.show()

if __name__ == "__main__":
    main()
