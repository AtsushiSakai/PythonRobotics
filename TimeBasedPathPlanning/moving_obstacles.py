import numpy as np
import random
import matplotlib.pyplot as plt

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

        return positions

show_animation = True

def main():
    grid = Grid(np.array([10, 10]))

    plt.figure()

    for t in range(0, grid.time_limit):
        plt.clf()

        if show_animation:  # pragma: no cover
            # TODO: iter is clunky. Should use np array
            ax = plt.axes()
            ax.set_xlim(0, grid.grid_size[0])
            ax.set_ylim(0, grid.grid_size[1])

            for (obs_idx, obs_path) in enumerate(grid.obstacle_paths):
                obs_pos = obs_path[t]
                # plt.plot(obs_pos[0], obs_pos[1], "xr")
                circle = plt.Circle((obs_pos[0], obs_pos[1]), 0.2)
                ax.add_patch(circle)
            plt.grid(True)
            plt.pause(0.3)

# TODO: better animation closing
# fig, ax = plt.subplots()
# line, = ax.plot([], [])
# ax.set_xlim(0, 10)
# ax.set_ylim(-1, 1)

# def init():
#     line.set_data([], [])
#     return line,

# def animate(i):
#     x = [0, 10]
#     y = [0, i % 2 * 2 - 1]
#     line.set_data(x, y)
#     return line,

# ani = animation.FuncAnimation(fig, animate, init_func=init, frames=100, interval=20, blit=True)

# def close_event(evt):
#     ani.event_source.stop()
#     plt.close(fig)

if __name__ == '__main__':
    main()