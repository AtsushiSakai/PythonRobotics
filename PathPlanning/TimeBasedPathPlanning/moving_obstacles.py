import numpy as np
import random
import matplotlib.pyplot as plt
import matplotlib.animation as animation

class Position:
    x: int
    y: int

    def __init__(self, x: int, y: int):
        self.x = x
        self.y = y

    def as_ndarray(self) -> np.ndarray[int, int]:
        return np.array([self.x, self.y])
    
    def __add__(self, other):
        if isinstance(other, Position):
            return Position(self.x + other.x, self.y + other.y)
        raise NotImplementedError(f"Addition not supported for Position and {type(other)}")
    
    def __sub__(self, other):
        if isinstance(other, Position):
            return Position(self.x - other.x, self.y - other.y)
        raise NotImplementedError(f"Subtraction not supported for Position and {type(other)}")

    def __eq__(self, other):
        if isinstance(other, Position):
            return self.x == other.x and self.y == other.y
        return False

    def __repr__(self):
        return f"Position({self.x}, {self.y})"

class Grid():
    
    # Set in constructor
    grid_size = None
    grid = None
    obstacle_paths = []
    # Obstacles will never occupy these points. Useful to avoid impossible scenarios
    obstacle_avoid_points = []

    # Problem definition
    time_limit = 100
    num_obstacles: int

    # Logging control
    verbose = False

    def __init__(self, grid_size: np.ndarray[int, int], num_obstacles: int = 2, obstacle_avoid_points: list[Position] = []):
        self.num_obstacles = num_obstacles
        self.obstacle_avoid_points = obstacle_avoid_points
        self.grid_size = grid_size
        self.grid = np.zeros((grid_size[0], grid_size[1], self.time_limit))

        if self.num_obstacles > self.grid_size[0] * self.grid_size[1]:
            raise Exception("Number of obstacles is greater than grid size!")

        for i in range(self.num_obstacles):
            self.obstacle_paths.append(self.generate_dynamic_obstacle(i+1))

    """
    Generate a dynamic obstacle following a random trajectory, and reserve its path in `self.grid`

    input:
        obs_idx (int): index of the obstacle. Used to reserve its path in `self.grid`
    
    output:
        list[np.ndarray[int, int]]: list of positions of the obstacle at each time step
    """
    def generate_dynamic_obstacle(self, obs_idx: int) -> list[Position]:

        # Sample until a free starting space is found
        initial_position = self.sample_random_position()
        while not self.valid_obstacle_position(initial_position, 0):
            initial_position = self.sample_random_position()

        positions = [initial_position]
        if self.verbose:
            print("Obstacle initial position: ", initial_position)

        # Encourage obstacles to mostly stay in place - too much movement leads to chaotic planning scenarios
        # that are not fun to watch
        weights = [0.05, 0.05, 0.05, 0.05, 0.8]
        diffs = [Position(0, 1), Position(0, -1), Position(1, 0), Position(-1, 0), Position(0, 0)]

        for t in range(1, self.time_limit-1):
            sampled_indices = np.random.choice(len(diffs), size=5, replace=False, p=weights)
            rand_diffs = [diffs[i] for i in sampled_indices]
            # rand_diffs = random.sample(diffs, k=len(diffs))

            valid_position = None
            for diff in rand_diffs:
                new_position = positions[-1] + diff

                if not self.valid_obstacle_position(new_position, t):
                    continue

                valid_position = new_position
                break

            # Impossible situation for obstacle - stay in place
            #   -> this can happen if another obstacle's path traps this one
            if valid_position is None:
                valid_position = positions[-1]

            # Reserve old & new position at this time step
            self.grid[positions[-1].x, positions[-1].y, t] = obs_idx
            self.grid[valid_position.x, valid_position.y, t] = obs_idx
            positions.append(valid_position)
    
        return positions

    """
    Check if the given position is valid at time t

    input:
        position (np.ndarray[int, int]): (x, y) position
        t (int): time step

    output:
        bool: True if position/time combination is valid, False otherwise
    """
    def valid_position(self, position: Position, t: int) -> bool:

        # Check if new position is in grid
        if not self.inside_grid_bounds(position):
            return False

        # Check if new position is not occupied at time t
        return self.grid[position.x, position.y, t] == 0
    
    """
    Returns True if the given position is valid at time t and is not in the set of obstacle_avoid_points
    """
    def valid_obstacle_position(self, position: Position, t: int) -> bool:
        return self.valid_position(position, t) and position not in self.obstacle_avoid_points
    
    """
    Returns True if the given position is within the grid's boundaries
    """
    def inside_grid_bounds(self, position: Position) -> bool:
        return position.x >= 0 and position.x < self.grid_size[0] and position.y >= 0 and position.y < self.grid_size[1]

    """
    Sample a random position that is within the grid's boundaries

    output:
        np.ndarray[int, int]: (x, y) position
    """
    def sample_random_position(self) -> Position:
        return Position(np.random.randint(0, self.grid_size[0]), np.random.randint(0, self.grid_size[1]))

show_animation = True

def main():
    grid = Grid(np.array([11, 11]))

    if not show_animation:
        return

    fig = plt.figure(figsize=(8, 7))
    ax = fig.add_subplot(autoscale_on=False, xlim=(0, grid.grid_size[0]-1), ylim=(0, grid.grid_size[1]-1))
    ax.set_aspect('equal')
    ax.grid()
    ax.set_xticks(np.arange(0, 11, 1))
    ax.set_yticks(np.arange(0, 11, 1))
    points, = ax.plot([], [], 'ro', ms=15)

    def get_frame(i):
        obs_x_points = []
        obs_y_points = []
        for obs_path in grid.obstacle_paths:
            obs_pos = obs_path[i]
            obs_x_points.append(obs_pos.x)
            obs_y_points.append(obs_pos.y)
        points.set_data(obs_x_points, obs_y_points)
        return points,

    _ani = animation.FuncAnimation(
        fig, get_frame, grid.time_limit-1, interval=500, blit=True, repeat=False)
    plt.show()

if __name__ == '__main__':
    main()