"""
Ant Colony Optimization (ACO) Path Planning

author: Anish (@anishk85)

See Wikipedia article (https://en.wikipedia.org/wiki/Ant_colony_optimization_algorithms)

References:
    - Dorigo, M.; Maniezzo, V.; Colorni, A. (1996). "Ant system: optimization by a 
      colony of cooperating agents". IEEE Transactions on Systems, Man, and 
      Cybernetics, Part B. 26 (1): 29–41.
    - Dorigo, M.; Stützle, T. (2004). "Ant Colony Optimization". MIT Press.

This implementation uses ACO to find collision-free paths in grid-based environments
by simulating pheromone trails laid down by artificial ants exploring the space.
"""

import math
import numpy as np
import matplotlib.pyplot as plt
import signal
import sys

# Add show_animation flag for consistency with other planners
show_animation = True

# ACO parameters
N_ANTS = 30  # Number of ants in colony
N_ITERATIONS = 100  # Number of iterations
ALPHA = 3.0  # Pheromone importance factor
BETA = 5.0  # Distance heuristic importance factor
RHO = 0.5  # Pheromone evaporation rate
Q = 100  # Pheromone deposit factor

def signal_handler(sig, frame):
    print("\nExiting...")
    plt.close("all")
    sys.exit(0)

signal.signal(signal.SIGINT, signal_handler)

class Node:
    """Represents a node in the grid."""

    def __init__(self, x, y):
        self.x = x  # Grid x coordinate
        self.y = y  # Grid y coordinate

    def __eq__(self, other):
        return self.x == other.x and self.y == other.y

    def __hash__(self):
        return hash((self.x, self.y))

class ACOPathPlanner:
    """
    Ant Colony Optimization path planner for grid-based environments.
    
    ACO simulates the behavior of real ants searching for food. Ants deposit
    pheromones on paths they traverse, and other ants are attracted to paths
    with higher pheromone concentrations. Over time, shorter paths accumulate
    more pheromones, leading the colony to converge on optimal solutions.
    
        Attributes:
        resolution: Grid resolution in meters
        min_x, max_x: X-axis boundaries
        min_y, max_y: Y-axis boundaries
        x_width, y_width: Grid dimensions
        obstacle_map: 2D boolean array marking obstacles
        pheromone: 2D array storing pheromone levels
        best_path: Best path found so far
        best_path_length: Length of best path
    """

    def __init__(self, ox, oy, resolution, start, goal):
        """
        Initialize ACO planner.
        
        Args:
            ox: List of x positions of obstacles [m]
            oy: List of y positions of obstacles [m]
            resolution: Grid resolution [m]
            start: Start position [x, y]
            goal: Goal position [x, y]
        """
        self.resolution = resolution
        self.min_x = min(ox) - 5
        self.min_y = min(oy) - 5
        self.max_x = max(ox) + 5
        self.max_y = max(oy) + 5
        self.x_width = int((self.max_x - self.min_x) / self.resolution)
        self.y_width = int((self.max_y - self.min_y) / self.resolution)

        self.obstacle_map = self.get_obstacle_map(ox, oy)
        self.motion = self.get_motion_model()

        self.start_node = Node(
            self.calc_xy_index(start[0], self.min_x),
            self.calc_xy_index(start[1], self.min_y),
        )
        self.goal_node = Node(
            self.calc_xy_index(goal[0], self.min_x),
            self.calc_xy_index(goal[1], self.min_y),
        )

        # Initialize pheromone matrix with small positive value
        self.pheromone = np.ones((self.x_width, self.y_width)) * 0.1

        # Store best solution
        self.best_path = None
        self.best_path_length = float("inf")

    def planning(self):
        """
        Execute ACO path planning algorithm.
        
        Returns:
            rx: List of x positions along the path
            ry: List of y positions along the path
        """
        for iteration in range(N_ITERATIONS):
            all_paths = []
            all_path_lengths = []

            # Each ant constructs a complete path
            for ant in range(N_ANTS):
                path = self.construct_solution()
                if path:
                    path_length = self.calc_path_length(path)
                    all_paths.append(path)
                    all_path_lengths.append(path_length)

                    # Update global best if this path is better
                    if path_length < self.best_path_length:
                        self.best_path = path
                        self.best_path_length = path_length

            # Update pheromone trails based on all paths found
            self.update_pheromones(all_paths, all_path_lengths)

            # Visualization
            if show_animation and iteration % 10 == 0:
                self.plot_pheromone_map(iteration)
                print(
                    f"Iteration {iteration}/{N_ITERATIONS}, "
                    f"Best length: {self.best_path_length * self.resolution:.2f}m"
                )

        if self.best_path is None:
            return [], []

        # Convert grid path to real coordinates
        rx = [self.calc_position(node.x, self.min_x) for node in self.best_path]
        ry = [self.calc_position(node.y, self.min_y) for node in self.best_path]

        return rx, ry

    def construct_solution(self):
        """
        Construct a solution path for one ant using probabilistic state transition.
        
        Returns:
            path: List of nodes representing the path, or None if no path found
        """
        current = Node(self.start_node.x, self.start_node.y)
        path = [current]
        visited = {(current.x, current.y)}

        max_steps = self.x_width * self.y_width
        steps = 0

        while steps < max_steps:
            steps += 1

            # Check if goal is reached
            if current.x == self.goal_node.x and current.y == self.goal_node.y:
                return path

            # Get valid unvisited neighbors
            neighbors = self.get_neighbors(current, visited)

            if not neighbors:
                return None  # Dead end - no valid moves

            # Select next node based on pheromone and heuristic
            next_node = self.select_next_node(current, neighbors)

            if next_node is None:
                return None

            path.append(next_node)
            visited.add((next_node.x, next_node.y))
            current = next_node

        return None  # Max steps reached without finding goal

    def get_neighbors(self, node, visited):
        """
        Get valid neighboring nodes (not visited, not obstacles, within bounds).
        
        Args:
            node: Current node
            visited: Set of visited node coordinates
            
        Returns:
            neighbors: List of valid neighbor nodes
        """
        neighbors = []

        for dx, dy in self.motion:
            x = node.x + dx
            y = node.y + dy

            # Check bounds
            if x < 0 or x >= self.x_width or y < 0 or y >= self.y_width:
                continue

            # Check obstacles
            if self.obstacle_map[x][y]:
                continue

            # Check if already visited
            if (x, y) in visited:
                continue

            neighbors.append(Node(x, y))

        return neighbors

    def select_next_node(self, current, neighbors):
        """
        Select next node based on pheromone trail and heuristic information.
        
        Uses the ACO probability formula:
        P[i,j] = (tau[i,j]^alpha * eta[i,j]^beta) / sum(tau^alpha * eta^beta)
        
        Args:
            current: Current node
            neighbors: List of candidate neighbor nodes
            
        Returns:
            selected_node: Next node to visit
        """
        if not neighbors:
            return None

        probabilities = []

        for neighbor in neighbors:
            # Pheromone level (tau)
            tau = self.pheromone[neighbor.x][neighbor.y]

            # Heuristic: inverse of distance to goal (eta)
            dist_to_goal = self.calc_heuristic(neighbor)
            eta = 1.0 / (dist_to_goal + 1e-10)  # Avoid division by zero

            # Probability calculation
            prob = (tau**ALPHA) * (eta**BETA)
            probabilities.append(prob)

        # Normalize probabilities
        total = sum(probabilities)
        if total == 0:
            # If all probabilities are 0, choose randomly
            return np.random.choice(neighbors)

        probabilities = [p / total for p in probabilities]

        # Select next node using roulette wheel selection
        selected_idx = np.random.choice(len(neighbors), p=probabilities)
        return neighbors[selected_idx]

    def calc_heuristic(self, node):
        """
        Calculate heuristic value (Euclidean distance to goal).
        
        Args:
            node: Node to evaluate
            
        Returns:
            distance: Euclidean distance to goal node
        """
        return math.hypot(node.x - self.goal_node.x, node.y - self.goal_node.y)

    def update_pheromones(self, all_paths, all_path_lengths):
        """
        Update pheromone levels: evaporation + deposition.
        
        Pheromone update rule:
        tau[i,j] = (1 - rho) * tau[i,j] + sum(delta_tau[i,j])
        
        Args:
            all_paths: List of paths found by all ants
            all_path_lengths: Corresponding path lengths
        """
        # Evaporation: reduce all pheromones
        self.pheromone *= 1 - RHO

        # Deposition: ants deposit pheromones on their paths
        for path, length in zip(all_paths, all_path_lengths):
            if path and length > 0:
                # Shorter paths get more pheromone
                deposit = Q / length
                for node in path:
                    self.pheromone[node.x][node.y] += deposit

    def calc_path_length(self, path):
        """
        Calculate total Euclidean path length.
        
        Args:
            path: List of nodes
            
        Returns:
            length: Total path length in grid units
        """
        length = 0.0
        for i in range(len(path) - 1):
            dx = path[i + 1].x - path[i].x
            dy = path[i + 1].y - path[i].y
            length += math.hypot(dx, dy)
        return length

    def calc_xy_index(self, position, min_pos):
        """Convert real position to grid index."""
        return int((position - min_pos) / self.resolution)

    def calc_position(self, index, min_pos):
        """Convert grid index to real position."""
        return index * self.resolution + min_pos

    def get_obstacle_map(self, ox, oy):
        """
        Create 2D boolean obstacle map from obstacle coordinates.
        
        Args:
            ox: List of obstacle x coordinates
            oy: List of obstacle y coordinates
            
        Returns:
            obstacle_map: 2D boolean array
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
        Define 8-directional movement model.
        
        Returns:
            motion: List of [dx, dy] movement vectors
        """
        motion = [
            [1, 0],  # right
            [0, 1],  # up
            [-1, 0],  # left
            [0, -1],  # down
            [1, 1],  # diagonal up-right
            [-1, 1],  # diagonal up-left
            [-1, -1],  # diagonal down-left
            [1, -1],  # diagonal down-right
        ]
        return motion

    def plot_pheromone_map(self, iteration):
        """
        Visualize pheromone distribution as heatmap.
        
        Args:
            iteration: Current iteration number
        """
        plt.clf()

        # Plot pheromone heatmap
        plt.imshow(
            self.pheromone.T,
            cmap="hot",
            origin="lower",
            extent=[self.min_x, self.max_x, self.min_y, self.max_y],
            alpha=0.7,
        )
        plt.colorbar(label="Pheromone Level")

        # Plot obstacles
        for x in range(self.x_width):
            for y in range(self.y_width):
                if self.obstacle_map[x][y]:
                    px = self.calc_position(x, self.min_x)
                    py = self.calc_position(y, self.min_y)
                    plt.plot(px, py, "sk", markersize=2)

        # Plot best path if exists
        if self.best_path:
            rx = [self.calc_position(node.x, self.min_x) for node in self.best_path]
            ry = [self.calc_position(node.y, self.min_y) for node in self.best_path]
            plt.plot(rx, ry, "-b", linewidth=2, label="Best Path")

        # Plot start and goal
        plt.plot(
            self.calc_position(self.start_node.x, self.min_x),
            self.calc_position(self.start_node.y, self.min_y),
            "og",
            markersize=10,
            label="Start",
        )
        plt.plot(
            self.calc_position(self.goal_node.x, self.min_x),
            self.calc_position(self.goal_node.y, self.min_y),
            "xr",
            markersize=10,
            label="Goal",
        )

        plt.title(f"ACO Path Planning - Iteration {iteration}")
        plt.xlabel("X [m]")
        plt.ylabel("Y [m]")
        plt.legend()
        plt.grid(True)
        plt.axis("equal")
        plt.pause(0.01)

def main():
    """Run ACO path planning algorithm demonstration."""
    print(__file__ + " start!!")

    # Start and goal positions
    sx = 10.0  # [m]
    sy = 10.0  # [m]
    gx = 50.0  # [m]
    gy = 50.0  # [m]
    grid_size = 2.0  # [m]

    # Set obstacle positions (create maze-like environment)
    ox, oy = [], []

    # Boundary walls
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
        plt.figure(figsize=(10, 8))
        plt.plot(ox, oy, ".k")
        plt.plot(sx, sy, "og", markersize=10)
        plt.plot(gx, gy, "xr", markersize=10)
        plt.grid(True)
        plt.axis("equal")
        plt.title("ACO Path Planning Environment")

    # Create ACO planner and run
    aco = ACOPathPlanner(ox, oy, grid_size, [sx, sy], [gx, gy])
    rx, ry = aco.planning()

    # Plot final result
    if show_animation:
        plt.figure(figsize=(10, 8))
        plt.plot(ox, oy, ".k", label="Obstacles")
        plt.plot(sx, sy, "og", markersize=10, label="Start")
        plt.plot(gx, gy, "xr", markersize=10, label="Goal")
        if rx:
            plt.plot(rx, ry, "-r", linewidth=2, label="ACO Path")
        plt.title("ACO Path Planning Result")
        plt.xlabel("X [m]")
        plt.ylabel("Y [m]")
        plt.legend()
        plt.grid(True)
        plt.axis("equal")
        plt.show()

    if rx:
        print("Path found!")
        print(f"Path length: {aco.best_path_length * grid_size:.2f} m")
    else:
        print("No path found!")

if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\nProgram interrupted by user")
        plt.close("all")
        sys.exit(0)