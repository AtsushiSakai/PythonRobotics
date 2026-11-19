"""
Fringe Search Path Planning

author: Anish (@anishk85)

See paper: Björnsson, Y.; Enzenberger, M.; Holte, R.; Schaeffer, J. (2005).
"Fringe Search: Beating A* at Pathfinding on Game Maps"

Reference:
    - https://webdocs.cs.ualberta.ca/~holte/Publications/fringe.pdf

Fringe Search is a memory-efficient graph search algorithm that combines
the benefits of iterative deepening with A*. It maintains a small "fringe"
of nodes and iteratively searches with increasing f-cost thresholds.

Key advantages:
- Lower memory usage than A* (no priority queue)
- Comparable or better performance on grid maps
- Simple implementation
"""

import math
import matplotlib.pyplot as plt

show_animation = True

class Node:
    """Node in the search grid."""

    def __init__(self, x, y, cost, parent_index):
        self.x = x
        self.y = y
        self.cost = cost  # g-cost (cost from start)
        self.parent_index = parent_index

    def __str__(self):
        return f"({self.x},{self.y}) cost:{self.cost} parent:{self.parent_index}"


class FringeSearch:
    """
    Fringe Search path planner.

    Uses iterative deepening with f-cost thresholds to find optimal paths
    while using minimal memory compared to A*.
    """

    def __init__(self, ox, oy, resolution, robot_radius):
        """
        Initialize Fringe Search planner.

        Args:
            ox: List of obstacle x coordinates [m]
            oy: List of obstacle y coordinates [m]
            resolution: Grid resolution [m]
            robot_radius: Robot radius [m]
        """
        self.resolution = resolution
        self.robot_radius = robot_radius
        self.min_x = min(ox)
        self.min_y = min(oy)
        self.max_x = max(ox)
        self.max_y = max(oy)
        self.x_width = int((self.max_x - self.min_x) / self.resolution)
        self.y_width = int((self.max_y - self.min_y) / self.resolution)
        self.obstacle_map = self.calc_obstacle_map(ox, oy)
        self.motion = self.get_motion_model()

    def planning(self, sx, sy, gx, gy):
        """
        Execute Fringe Search path planning.

        Args:
            sx: Start x position [m]
            sy: Start y position [m]
            gx: Goal x position [m]
            gy: Goal y position [m]

        Returns:
            rx: x position list of the final path
            ry: y position list of the final path
        """
        start_node = Node(
            self.calc_xy_index(sx, self.min_x),
            self.calc_xy_index(sy, self.min_y),
            0.0,
            -1,
        )
        goal_node = Node(
            self.calc_xy_index(gx, self.min_x),
            self.calc_xy_index(gy, self.min_y),
            0.0,
            -1,
        )

        # Initialize data structures
        fringe = [start_node]  # The fringe list
        cache = {}  # Node cache: index -> (node, f_cost, in_fringe)

        start_index = self.calc_index(start_node)
        h_start = self.calc_heuristic(start_node, goal_node)
        cache[start_index] = (start_node, start_node.cost + h_start, True)

        flimit = h_start  # Initial f-cost threshold
        found = False

        if show_animation:
            plt.figure(figsize=(10, 8))

        iteration = 0
        while not found and fringe:
            fmin = float('inf')  # Minimum f-cost exceeding current threshold

            # Iterate through fringe with current threshold
            fringe_copy = fringe.copy()
            fringe.clear()

            for node in fringe_copy:
                node_index = self.calc_index(node)

                if node_index not in cache:
                    continue

                _, f_cost, in_fringe = cache[node_index]

                if not in_fringe:
                    continue

                # If f-cost exceeds threshold, defer to next iteration
                if f_cost > flimit:
                    fmin = min(f_cost, fmin)
                    fringe.append(node)
                    continue

                # Goal test
                if node.x == goal_node.x and node.y == goal_node.y:
                    goal_node.parent_index = node.parent_index
                    goal_node.cost = node.cost
                    found = True
                    break

                # Mark as visited (remove from fringe)
                cache[node_index] = (node, f_cost, False)

                # Visualization
                if show_animation and iteration % 10 == 0:
                    self.plot_current_state(node, goal_node, cache)

                # Expand neighbors
                for motion in self.motion:
                    n_node = Node(
                        node.x + motion[0],
                        node.y + motion[1],
                        node.cost + motion[2],
                        node_index,
                    )
                    n_index = self.calc_index(n_node)

                    # Check validity
                    if not self.verify_node(n_node):
                        continue

                    # Calculate costs
                    g_cost = n_node.cost
                    h_cost = self.calc_heuristic(n_node, goal_node)
                    f_cost = g_cost + h_cost

                    if n_index in cache:
                        cached_node, cached_f, cached_in_fringe = cache[n_index]

                        # Update if we found a better path
                        if g_cost < cached_node.cost:
                            cache[n_index] = (n_node, f_cost, True)
                            if not cached_in_fringe:
                                fringe.append(n_node)
                    else:
                        # New node
                        cache[n_index] = (n_node, f_cost, True)
                        fringe.append(n_node)

                iteration += 1

            if not found:
                # Update threshold for next iteration
                flimit = fmin

        # Reconstruct path
        rx, ry = self.calc_final_path(goal_node, cache)

        if show_animation:
            self.plot_final_path(rx, ry, sx, sy, gx, gy)

        return rx, ry

    def calc_final_path(self, goal_node, cache):
        """
        Reconstruct path from goal to start.

        Args:
            goal_node: Goal node
            cache: Node cache dictionary

        Returns:
            rx: x position list
            ry: y position list
        """
        rx = [self.calc_position(goal_node.x, self.min_x)]
        ry = [self.calc_position(goal_node.y, self.min_y)]
        parent_index = goal_node.parent_index

        while parent_index != -1:
            if parent_index not in cache:
                break
            n, _, _ = cache[parent_index]
            rx.append(self.calc_position(n.x, self.min_x))
            ry.append(self.calc_position(n.y, self.min_y))
            parent_index = n.parent_index

        return rx, ry

    def calc_heuristic(self, node, goal):
        """
        Calculate heuristic (Euclidean distance).

        Args:
            node: Current node
            goal: Goal node

        Returns:
            Heuristic value
        """
        return math.hypot(node.x - goal.x, node.y - goal.y)

    def calc_index(self, node):
        """Calculate grid index."""
        return node.y * self.x_width + node.x

    def calc_xy_index(self, position, min_pos):
        """Convert position to grid index."""
        return int((position - min_pos) / self.resolution)

    def calc_position(self, index, min_pos):
        """Convert grid index to position."""
        return index * self.resolution + min_pos

    def verify_node(self, node):
        """Check if node is valid (within bounds and not obstacle)."""
        if node.x < 0 or node.x >= self.x_width:
            return False
        if node.y < 0 or node.y >= self.y_width:
            return False
        if self.obstacle_map[node.x][node.y]:
            return False
        return True

    def calc_obstacle_map(self, ox, oy):
        """
        Create obstacle map with inflation.

        Args:
            ox: Obstacle x coordinates
            oy: Obstacle y coordinates

        Returns:
            2D boolean obstacle map
        """
        obstacle_map = [
            [False for _ in range(self.y_width)] for _ in range(self.x_width)
        ]

        for x, y in zip(ox, oy):
            ix = self.calc_xy_index(x, self.min_x)
            iy = self.calc_xy_index(y, self.min_y)
            if 0 <= ix < self.x_width and 0 <= iy < self.y_width:
                obstacle_map[ix][iy] = True

        return obstacle_map

    @staticmethod
    def get_motion_model():
        """
        Define 8-directional motion model.

        Returns:
            List of [dx, dy, cost] movements
        """
        motion = [
            [1, 0, 1.0],
            [0, 1, 1.0],
            [-1, 0, 1.0],
            [0, -1, 1.0],
            [-1, -1, math.sqrt(2)],
            [-1, 1, math.sqrt(2)],
            [1, -1, math.sqrt(2)],
            [1, 1, math.sqrt(2)],
        ]
        return motion

    def plot_current_state(self, current, goal, cache):
        """Visualize current search state."""
        plt.clf()

        # Plot obstacles
        for x in range(self.x_width):
            for y in range(self.y_width):
                if self.obstacle_map[x][y]:
                    px = self.calc_position(x, self.min_x)
                    py = self.calc_position(y, self.min_y)
                    plt.plot(px, py, "sk", markersize=2)

        # Plot visited nodes
        for node_index, (node, _, in_fringe) in cache.items():
            if not in_fringe:
                px = self.calc_position(node.x, self.min_x)
                py = self.calc_position(node.y, self.min_y)
                plt.plot(px, py, "xc", markersize=1)

        # Plot current node
        px = self.calc_position(current.x, self.min_x)
        py = self.calc_position(current.y, self.min_y)
        plt.plot(px, py, "og")

        # Plot goal
        px = self.calc_position(goal.x, self.min_x)
        py = self.calc_position(goal.y, self.min_y)
        plt.plot(px, py, "xr")

        plt.grid(True)
        plt.axis("equal")
        plt.pause(0.001)

    def plot_final_path(self, rx, ry, sx, sy, gx, gy):
        """Visualize final path."""
        plt.plot(rx, ry, "-r", linewidth=2, label="Fringe Search Path")
        plt.plot(sx, sy, "og", markersize=10, label="Start")
        plt.plot(gx, gy, "xr", markersize=10, label="Goal")
        plt.legend()
        plt.grid(True)
        plt.axis("equal")
        plt.title("Fringe Search Result")
        plt.xlabel("X [m]")
        plt.ylabel("Y [m]")
        plt.show()


def main():
    """Run Fringe Search demonstration."""
    print(__file__ + " start!!")

    # Start and goal positions
    sx = 10.0  # [m]
    sy = 10.0  # [m]
    gx = 50.0  # [m]
    gy = 50.0  # [m]
    grid_size = 2.0  # [m]
    robot_radius = 1.0  # [m]

    # Set obstacle positions
    ox, oy = [], []

    # Boundary
    for i in range(0, 60):
        ox.append(i)
        oy.append(0.0)
    for i in range(0, 60):
        ox.append(60.0)
        oy.append(i)
    for i in range(0, 61):
        ox.append(i)
        oy.append(60.0)
    for i in range(0, 61):
        ox.append(0.0)
        oy.append(i)

    # Internal obstacles
    for i in range(0, 40):
        ox.append(20.0)
        oy.append(i)
    for i in range(0, 40):
        ox.append(40.0)
        oy.append(60.0 - i)

    if show_animation:
        plt.plot(ox, oy, ".k")
        plt.plot(sx, sy, "og")
        plt.plot(gx, gy, "xr")
        plt.grid(True)
        plt.axis("equal")

    fringe = FringeSearch(ox, oy, grid_size, robot_radius)
    rx, ry = fringe.planning(sx, sy, gx, gy)

    if rx:
        print("Path found!")
        print(f"Path length: {len(rx)} nodes")
    else:
        print("No path found!")


if __name__ == "__main__":
    main()
