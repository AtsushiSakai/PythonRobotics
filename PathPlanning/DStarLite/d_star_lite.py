"""
D* Lite grid planning
author: Taha Zahid (@TahaZahid05)
Original author: vss2sn

Link to papers:
D* Lite (Link: http://idm-lab.org/bib/abstracts/papers/aaai02b.pdf)
Improved Fast Replanning for Robot Navigation in Unknown Terrain
(Link: http://www.cs.cmu.edu/~maxim/files/dlite_icra02.pdf)

Optimized using heapq (priority queue) with lazy deletion for O(log n)
priority queue operations.
"""
import heapq
import math
import matplotlib.pyplot as plt
import random
import numpy as np

show_animation = True
pause_time = 0.001
p_create_random_obstacle = 0


class Node:
    def __init__(self, x: int = 0, y: int = 0, cost: float = 0.0):
        self.x = x
        self.y = y
        self.cost = cost

    def __lt__(self, other):
        return False

    def __eq__(self, other):
        if not isinstance(other, Node):
            return False
        return self.x == other.x and self.y == other.y

    def __hash__(self):
        return hash((self.x, self.y))


def add_coordinates(node1: Node, node2: Node):
    new_node = Node()
    new_node.x = node1.x + node2.x
    new_node.y = node1.y + node2.y
    new_node.cost = node1.cost + node2.cost
    return new_node


def compare_coordinates(node1: Node, node2: Node):
    return node1.x == node2.x and node1.y == node2.y


class DStarLite:

    # Please adjust the heuristic function (h) if you change the list of
    # possible motions
    motions = [
        Node(1, 0, 1),
        Node(0, 1, 1),
        Node(-1, 0, 1),
        Node(0, -1, 1),
        Node(1, 1, math.sqrt(2)),
        Node(1, -1, math.sqrt(2)),
        Node(-1, 1, math.sqrt(2)),
        Node(-1, -1, math.sqrt(2))
    ]

    def __init__(self, ox: list, oy: list):
        # Ensure that within the algorithm implementation all node coordinates
        # are indices in the grid and extend
        # from 0 to abs(<axis>_max - <axis>_min)
        self.x_min_world = int(min(ox))
        self.y_min_world = int(min(oy))
        self.x_max = int(abs(max(ox) - self.x_min_world))
        self.y_max = int(abs(max(oy) - self.y_min_world))
        self.obstacles = [Node(x - self.x_min_world, y - self.y_min_world)
                          for x, y in zip(ox, oy)]
        self.obstacles_xy = {(obstacle.x, obstacle.y) for obstacle in self.obstacles}
        self.start = Node(0, 0)
        self.goal = Node(0, 0)
        # Priority queue implemented with heapq for O(log n) operations
        self.U: list = []  # Min-heap for open set
        self.entry_finder: dict = {}  # Maps nodes to heap entries for O(1) lookup
        self.counter = 0  # Unique sequence count for tie-breaking
        self.km = 0.0
        self.rhs = self.create_grid(float("inf"))
        self.g = self.create_grid(float("inf"))
        self.detected_obstacles_xy: set[tuple[int, int]] = set()
        if show_animation:
            self.detected_obstacles_for_plotting_x = list()  # type: ignore
            self.detected_obstacles_for_plotting_y = list()  # type: ignore
        self.initialized = False

    def create_grid(self, val: float):
        return np.full((self.x_max, self.y_max), val)

    def is_obstacle(self, node: Node):
        is_in_obstacles = (node.x, node.y) in self.obstacles_xy
        is_in_detected_obstacles = (node.x, node.y) in self.detected_obstacles_xy
        return is_in_obstacles or is_in_detected_obstacles

    def c(self, node1: Node, node2: Node):
        if self.is_obstacle(node2):
            # Attempting to move from or to an obstacle
            return math.inf
        new_node = Node(node1.x-node2.x, node1.y-node2.y)
        detected_motion = list(filter(lambda motion:
                                      compare_coordinates(motion, new_node),
                                      self.motions))
        return detected_motion[0].cost

    def h(self, s: Node):
        # Cannot use the 2nd euclidean norm as this might sometimes generate
        # heuristics that overestimate the cost, making them inadmissible,
        # due to rounding errors etc (when combined with calculate_key)
        # To be admissible heuristic should
        # never overestimate the cost of a move
        # hence not using the line below
        # return math.hypot(self.start.x - s.x, self.start.y - s.y)

        # Below is the same as 1; modify if you modify the cost of each move in
        # motion
        # return max(abs(self.start.x - s.x), abs(self.start.y - s.y))
        return 1

    def calculate_key(self, s: Node) -> list[float]:
        return [min(self.g[s.x][s.y], self.rhs[s.x][s.y]) + self.h(s) + self.km,
                min(self.g[s.x][s.y], self.rhs[s.x][s.y])]

    def is_valid(self, node: Node):
        if 0 <= node.x < self.x_max and 0 <= node.y < self.y_max:
            return True
        return False

    def get_neighbours(self, u: Node):
        return [add_coordinates(u, motion) for motion in self.motions
                if self.is_valid(add_coordinates(u, motion))]

    def pred(self, u: Node):
        # Grid, so each vertex is connected to the ones around it
        return self.get_neighbours(u)

    def succ(self, u: Node):
        # Grid, so each vertex is connected to the ones around it
        return self.get_neighbours(u)

    def push(self, task: Node, priority: list):
        """
        Add a new node to priority queue or update its priority.
        Uses lazy deletion pattern for efficient priority updates.

        Args:
            task: Node to add/update
            priority: Priority key [f-value, g-value]
        """
        if task in self.entry_finder:
            self.remove(task)
        count = self.counter
        self.counter += 1
        entry = [priority, count, task]
        self.entry_finder[task] = entry
        heapq.heappush(self.U, entry)

    def remove(self, task: Node):
        """
        Mark an existing task as removed (lazy deletion).
        The actual removal from heap happens during pop/peek operations.

        Args:
            task: Node to mark as removed
        """
        entry = self.entry_finder.pop(task)
        entry[-1] = None  # Mark as removed

    def pop(self):
        """
        Remove and return the lowest priority task.
        Skips over entries marked as removed (lazy deletion).

        Returns:
            tuple: (task Node, priority list)

        Raises:
            KeyError: If heap is empty
        """
        while self.U:
            priority, count, task = heapq.heappop(self.U)
            if task is not None:
                del self.entry_finder[task]
                return task, priority
        raise KeyError("empty heap")

    def contains(self, task: Node):
        """
        Check if a node is in the priority queue.

        Args:
            task: Node to check

        Returns:
            bool: True if node is in queue
        """
        return task in self.entry_finder

    def peek(self):
        """
        Return the lowest priority task without removing it.
        Cleans up entries marked as removed.

        Returns:
            tuple: (task Node or None, priority list)
        """
        if not self.U:
            return None, [float('inf'), float('inf')]

        while self.U:
            entry = self.U[0]
            priority, count, task = entry

            if task is not None:
                return task, priority

            heapq.heappop(self.U)  # Remove invalid entries

        return None, [float('inf'), float('inf')]

    def key_less_than(self, k1: list, k2: list):
        """
        Lexicographical comparison of priority keys.

        Args:
            k1: First key [f-value, g-value]
            k2: Second key [f-value, g-value]

        Returns:
            bool: True if k1 < k2 lexicographically
        """
        return k1[0] < k2[0] or (k1[0] == k2[0] and k1[1] < k2[1])

    def initialize(self, start: Node, goal: Node):
        self.start.x = start.x - self.x_min_world
        self.start.y = start.y - self.y_min_world
        self.goal.x = goal.x - self.x_min_world
        self.goal.y = goal.y - self.y_min_world
        if not self.initialized:
            self.initialized = True
            print('Initializing')
            self.U = []
            self.entry_finder.clear()
            self.counter = 0
            self.km = 0.0
            self.rhs = self.create_grid(math.inf)
            self.g = self.create_grid(math.inf)
            self.rhs[self.goal.x][self.goal.y] = 0
            self.push(self.goal, self.calculate_key(self.goal))
            self.detected_obstacles_xy = set()

    def update_vertex(self, u: Node):
        if not compare_coordinates(u, self.goal):
            self.rhs[u.x][u.y] = min([self.c(u, sprime) +
                                      self.g[sprime.x][sprime.y]
                                      for sprime in self.succ(u)])
        if self.g[u.x][u.y] != self.rhs[u.x][u.y]:
            if self.contains(u):
                self.remove(u)
            self.push(u, self.calculate_key(u))
        elif self.g[u.x][u.y] == self.rhs[u.x][u.y] and self.contains(u):
            self.remove(u)

    def compute_shortest_path(self):
        while True:
            task, k_old = self.peek()
            if task is None:
                break

            k_start = self.calculate_key(self.start)

            # Stop condition: Start is consistent AND top of heap >= Start Key
            if (not self.key_less_than(k_old, k_start) and
                self.rhs[self.start.x][self.start.y] == self.g[self.start.x][self.start.y]):
                break

            u, k_old = self.pop()
            k_new = self.calculate_key(u)

            if self.key_less_than(k_old, k_new):
                # Node priority has improved, re-insert
                self.push(u, k_new)

            elif self.g[u.x][u.y] > self.rhs[u.x][u.y]:
                # Overconsistent (path found/improved): Propagate cost to neighbors
                self.g[u.x][u.y] = self.rhs[u.x][u.y]

                neighbors_lst = self.pred(u)

                for curr in neighbors_lst:
                    if curr != self.goal:
                        edge_cost = self.c(curr, u)
                        self.rhs[curr.x][curr.y] = min(self.rhs[curr.x][curr.y],
                                                    edge_cost + self.g[u.x][u.y])
                    self.update_vertex(curr)
            else:
                # Underconsistent (obstacle detected): Reset g to infinity and re-evaluate neighbors
                g_old = self.g[u.x][u.y]
                self.g[u.x][u.y] = math.inf

                neighbors_lst = self.pred(u)

                for curr in (neighbors_lst + [u]):
                    if curr == u:
                        # When curr is u itself, recalculate rhs for u
                        if curr != self.goal:
                            temp_rhs = float('inf')

                            curr_neighbors_lst = self.succ(curr)

                            for j in curr_neighbors_lst:
                                edge_cost = self.c(curr, j)
                                temp_rhs = min(temp_rhs, (edge_cost + self.g[j.x][j.y]))

                            self.rhs[curr.x][curr.y] = temp_rhs
                    else:
                        # For neighbors of u, check if they need rhs recalculation
                        edge_cost = self.c(curr, u)
                        if self.rhs[curr.x][curr.y] == (edge_cost + g_old):
                            if curr != self.goal:
                                temp_rhs = float('inf')

                                curr_neighbors_lst = self.succ(curr)

                                for j in curr_neighbors_lst:
                                    edge_cost = self.c(curr, j)
                                    temp_rhs = min(temp_rhs, (edge_cost + self.g[j.x][j.y]))

                                self.rhs[curr.x][curr.y] = temp_rhs

                    self.update_vertex(curr)

    def detect_changes(self):
        changed_vertices = list()
        if len(self.spoofed_obstacles) > 0:
            for spoofed_obstacle in self.spoofed_obstacles[0]:
                if compare_coordinates(spoofed_obstacle, self.start) or \
                   compare_coordinates(spoofed_obstacle, self.goal):
                    continue
                changed_vertices.append(spoofed_obstacle)
                self.detected_obstacles_xy.add((spoofed_obstacle.x, spoofed_obstacle.y))
                if show_animation:
                    self.detected_obstacles_for_plotting_x.append(
                        spoofed_obstacle.x + self.x_min_world)
                    self.detected_obstacles_for_plotting_y.append(
                        spoofed_obstacle.y + self.y_min_world)
                    plt.plot(self.detected_obstacles_for_plotting_x,
                             self.detected_obstacles_for_plotting_y, ".k")
                    plt.pause(pause_time)
            self.spoofed_obstacles.pop(0)

        # Allows random generation of obstacles
        random.seed()
        if random.random() > 1 - p_create_random_obstacle:
            x = random.randint(0, self.x_max - 1)
            y = random.randint(0, self.y_max - 1)
            new_obs = Node(x, y)
            if compare_coordinates(new_obs, self.start) or \
               compare_coordinates(new_obs, self.goal):
                return changed_vertices
            changed_vertices.append(Node(x, y))
            self.detected_obstacles_xy.add((x, y))
            if show_animation:
                self.detected_obstacles_for_plotting_x.append(x +
                                                              self.x_min_world)
                self.detected_obstacles_for_plotting_y.append(y +
                                                              self.y_min_world)
                plt.plot(self.detected_obstacles_for_plotting_x,
                         self.detected_obstacles_for_plotting_y, ".k")
                plt.pause(pause_time)
        return changed_vertices

    def compute_current_path(self):
        path = list()
        current_point = Node(self.start.x, self.start.y)
        while not compare_coordinates(current_point, self.goal):
            path.append(current_point)
            current_point = min(self.succ(current_point),
                                key=lambda sprime:
                                self.c(current_point, sprime) +
                                self.g[sprime.x][sprime.y])
        path.append(self.goal)
        return path

    def compare_paths(self, path1: list, path2: list):
        if len(path1) != len(path2):
            return False
        for node1, node2 in zip(path1, path2):
            if not compare_coordinates(node1, node2):
                return False
        return True

    def display_path(self, path: list, colour: str, alpha: float = 1.0):
        px = [(node.x + self.x_min_world) for node in path]
        py = [(node.y + self.y_min_world) for node in path]
        drawing = plt.plot(px, py, colour, alpha=alpha)
        plt.pause(pause_time)
        return drawing

    def main(self, start: Node, goal: Node,
             spoofed_ox: list, spoofed_oy: list):
        self.spoofed_obstacles = [[Node(x - self.x_min_world,
                                        y - self.y_min_world)
                                   for x, y in zip(rowx, rowy)]
                                  for rowx, rowy in zip(spoofed_ox, spoofed_oy)
                                  ]
        pathx = []
        pathy = []
        self.initialize(start, goal)
        last = self.start
        self.compute_shortest_path()
        pathx.append(self.start.x + self.x_min_world)
        pathy.append(self.start.y + self.y_min_world)

        if show_animation:
            current_path = self.compute_current_path()
            previous_path = current_path.copy()
            previous_path_image = self.display_path(previous_path, ".c",
                                                    alpha=0.3)
            current_path_image = self.display_path(current_path, ".c")

        while not compare_coordinates(self.goal, self.start):
            if self.g[self.start.x][self.start.y] == math.inf:
                print("No path possible")
                return False, pathx, pathy
            self.start = min(self.succ(self.start),
                             key=lambda sprime:
                             self.c(self.start, sprime) +
                             self.g[sprime.x][sprime.y])
            pathx.append(self.start.x + self.x_min_world)
            pathy.append(self.start.y + self.y_min_world)
            if show_animation:
                current_path.pop(0)
                plt.plot(pathx, pathy, "-r")
                plt.pause(pause_time)
            changed_vertices = self.detect_changes()
            if len(changed_vertices) != 0:
                print("New obstacle detected")
                self.km += self.h(last)
                last = self.start
                for u in changed_vertices:
                    if compare_coordinates(u, self.start):
                        continue
                    self.rhs[u.x][u.y] = math.inf
                    self.g[u.x][u.y] = math.inf
                    self.update_vertex(u)
                self.compute_shortest_path()

                if show_animation:
                    new_path = self.compute_current_path()
                    if not self.compare_paths(current_path, new_path):
                        current_path_image[0].remove()
                        previous_path_image[0].remove()
                        previous_path = current_path.copy()
                        current_path = new_path.copy()
                        previous_path_image = self.display_path(previous_path,
                                                                ".c",
                                                                alpha=0.3)
                        current_path_image = self.display_path(current_path,
                                                               ".c")
                        plt.pause(pause_time)
        print("Path found")
        return True, pathx, pathy


def main():

    # start and goal position
    sx = 10  # [m]
    sy = 10  # [m]
    gx = 50  # [m]
    gy = 50  # [m]

    # set obstacle positions
    ox, oy = [], []
    for i in range(-10, 60):
        ox.append(i)
        oy.append(-10.0)
    for i in range(-10, 60):
        ox.append(60.0)
        oy.append(i)
    for i in range(-10, 61):
        ox.append(i)
        oy.append(60.0)
    for i in range(-10, 61):
        ox.append(-10.0)
        oy.append(i)
    for i in range(-10, 40):
        ox.append(20.0)
        oy.append(i)
    for i in range(0, 40):
        ox.append(40.0)
        oy.append(60.0 - i)

    if show_animation:
        plt.plot(ox, oy, ".k")
        plt.plot(sx, sy, "og")
        plt.plot(gx, gy, "xb")
        plt.grid(True)
        plt.axis("equal")
        label_column = ['Start', 'Goal', 'Path taken',
                        'Current computed path', 'Previous computed path',
                        'Obstacles']
        columns = [plt.plot([], [], symbol, color=colour, alpha=alpha)[0]
                   for symbol, colour, alpha in [['o', 'g', 1],
                                                 ['x', 'b', 1],
                                                 ['-', 'r', 1],
                                                 ['.', 'c', 1],
                                                 ['.', 'c', 0.3],
                                                 ['.', 'k', 1]]]
        plt.legend(columns, label_column, bbox_to_anchor=(1, 1), title="Key:",
                   fontsize="xx-small")
        plt.plot()
        plt.pause(pause_time)

    # Obstacles discovered at time = row
    # time = 1, obstacles discovered at (0, 2), (9, 2), (4, 0)
    # time = 2, obstacles discovered at (0, 1), (7, 7)
    # ...
    # when the spoofed obstacles are:
    # spoofed_ox = [[0, 9, 4], [0, 7], [], [], [], [], [], [5]]
    # spoofed_oy = [[2, 2, 0], [1, 7], [], [], [], [], [], [4]]

    # Reroute
    # spoofed_ox = [[], [], [], [], [], [], [], [40 for _ in range(10, 21)]]
    # spoofed_oy = [[], [], [], [], [], [], [], [i for i in range(10, 21)]]

    # Obstacles that demostrate large rerouting
    spoofed_ox = [[], [], [],
                  [i for i in range(0, 21)] + [0 for _ in range(0, 20)]]
    spoofed_oy = [[], [], [],
                  [20 for _ in range(0, 21)] + [i for i in range(0, 20)]]

    dstarlite = DStarLite(ox, oy)
    dstarlite.main(Node(x=sx, y=sy), Node(x=gx, y=gy),
                   spoofed_ox=spoofed_ox, spoofed_oy=spoofed_oy)


if __name__ == "__main__":
    main()
