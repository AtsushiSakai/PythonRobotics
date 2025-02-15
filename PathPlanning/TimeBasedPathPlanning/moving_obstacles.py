import numpy as np
import random
import matplotlib.pyplot as plt
import matplotlib.animation as animation
class Grid():
    
    # Set in constructor
    grid_size = None
    grid = None
    obstacle_paths = []

    # Problem definition
    time_limit = 100
    num_obstacles = 2

    # Logging control
    verbose = False

    def __init__(self, grid_size: np.ndarray[int, int]):
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
    def generate_dynamic_obstacle(self, obs_idx: int) -> list[np.ndarray[int, int]]:

        # Sample until a free starting space is found
        initial_position = self.sample_random_position()
        while not self.valid_position(initial_position, 0):
            initial_position = self.sample_random_position()

        positions = [initial_position]
        if self.verbose:
            print("Obstacle initial position: ", initial_position)
        
        diffs = [np.array([0, 1]), np.array([0, -1]), np.array([1, 0]), np.array([-1, 0]), np.array([0, 0])]
        weights = [0.125, 0.125, 0.125, 0.125, 0.5] 

        for t in range(1, self.time_limit-1):
            rand_diffs = random.sample(diffs, k=5)

            valid_position = None
            for diff in rand_diffs:
                new_position = positions[-1] + diff

                if not self.valid_position(new_position, t):
                    continue

                valid_position = new_position
                break

            # Impossible situation for obstacle - stay in place
            #   -> this can happen if another obstacle's path traps this one
            if valid_position is None:
                valid_position = positions[-1]

            # Reserve old & new position at this time step
            positions.append(new_position)
            self.grid[positions[-2][0], positions[-2][1], t] = obs_idx
            self.grid[new_position[0], new_position[1], t] = obs_idx
    
        return positions

    """
    Check if the given position is valid at time t

    input:
        position (np.ndarray[int, int]): (x, y) position
        t (int): time step

    output:
        bool: True if position/time combination is valid, False otherwise
    """
    def valid_position(self, position, t) -> bool:

        # Check if new position is in grid
        if position[0] < 0 or position[0] >= self.grid_size[0] or position[1] < 0 or position[1] >= self.grid_size[1]:
            return False

        # Check if new position is not occupied at time t
        return self.grid[position[0], position[1], t] == 0

    """
    Sample a random position that is within the grid's boundaries

    output:
        np.ndarray[int, int]: (x, y) position
    """
    def sample_random_position(self) -> np.ndarray[int, int]:
        return np.array([np.random.randint(0, self.grid_size[0]), np.random.randint(0, self.grid_size[1])])

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
            obs_x_points.append(obs_pos[0])
            obs_y_points.append(obs_pos[1])
        points.set_data(obs_x_points, obs_y_points)
        return points,

    _ani = animation.FuncAnimation(
        fig, get_frame, grid.time_limit-1, interval=500, blit=True, repeat=False)
    plt.show()

if __name__ == '__main__':
    main()