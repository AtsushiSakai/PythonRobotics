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

    def __init__(self, grid_size: np.ndarray[int, int]):
        self.grid_size = grid_size
        self.grid = np.zeros((grid_size[0], grid_size[1], self.time_limit))

        for i in range(1, self.num_obstacles+1):
            self.obstacle_paths.append(self.generate_dynamic_obstacle(i))

    def generate_dynamic_obstacle(self, obs_idx: int) -> list[np.ndarray[int, int]]:
        # TODO: dont spawn on another obstacle
        initial_position = (np.random.randint(0, self.grid_size[0]), np.random.randint(0, self.grid_size[1]))
        positions = [initial_position]
        print("Initial position: ", initial_position)
        
        diffs = [np.array([0, 1]), np.array([0, -1]), np.array([1, 0]), np.array([-1, 0]), np.array([0, 0])]

        for t in range(1, self.time_limit-1):
            random.shuffle(diffs)
            for diff in diffs:
                new_position = positions[-1] + diff

                # Check if new position is in grid
                if new_position[0] < 0 or new_position[0] >= self.grid_size[0] or new_position[1] < 0 or new_position[1] >= self.grid_size[1]:
                    continue

                # Check if new position occupied by another obstacle
                if self.grid[new_position[0], new_position[1], t] == 0:
                    positions.append(new_position)
                    self.grid[new_position[0], new_position[1], t] = obs_idx
                    break

                # Impossible situation for obstacle - stay in place
                print("Impossible situation for obstacle!")
                positions.append(positions[-1])

        print('obs path len: ', len(positions))
        return positions

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
        for (_obs_idx, obs_path) in enumerate(grid.obstacle_paths):
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