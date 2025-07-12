"""
This file implements a grid with a 3d reservation matrix with dimensions for x, y, and time. There
is also infrastructure to generate dynamic obstacles that move around the grid. The obstacles' paths
are stored in the reservation matrix on creation.
"""
import numpy as np
import matplotlib.pyplot as plt
from enum import Enum
from dataclasses import dataclass
from PathPlanning.TimeBasedPathPlanning.Node import NodePath, Position

@dataclass
class Interval:
    start_time: int
    end_time: int

class ObstacleArrangement(Enum):
    # Random obstacle positions and movements
    RANDOM = 0
    # Obstacles start in a line in y at center of grid and move side-to-side in x
    ARRANGEMENT1 = 1
    # Static obstacle arrangement
    NARROW_CORRIDOR = 2

"""
Generates a 2d numpy array with lists for elements.
"""
def empty_2d_array_of_lists(x: int, y: int) -> np.ndarray:
    arr = np.empty((x, y), dtype=object)
    # assign each element individually - np.full creates references to the same list
    arr[:] = [[[] for _ in range(y)] for _ in range(x)]
    return arr

class Grid:
    # Set in constructor
    grid_size: np.ndarray
    reservation_matrix: np.ndarray
    obstacle_paths: list[list[Position]] = []
    # Obstacles will never occupy these points. Useful to avoid impossible scenarios
    obstacle_avoid_points: list[Position] = []

    # Number of time steps in the simulation
    time_limit: int

    # Logging control
    verbose = False

    def __init__(
        self,
        grid_size: np.ndarray,
        num_obstacles: int = 40,
        obstacle_avoid_points: list[Position] = [],
        obstacle_arrangement: ObstacleArrangement = ObstacleArrangement.RANDOM,
        time_limit: int = 100,
    ):
        self.obstacle_avoid_points = obstacle_avoid_points
        self.time_limit = time_limit
        self.grid_size = grid_size
        self.reservation_matrix = np.zeros((grid_size[0], grid_size[1], self.time_limit))

        if num_obstacles > self.grid_size[0] * self.grid_size[1]:
            raise Exception("Number of obstacles is greater than grid size!")

        if obstacle_arrangement == ObstacleArrangement.RANDOM:
            self.obstacle_paths = self.generate_dynamic_obstacles(num_obstacles)
        elif obstacle_arrangement == ObstacleArrangement.ARRANGEMENT1:
            self.obstacle_paths = self.obstacle_arrangement_1(num_obstacles)
        elif obstacle_arrangement == ObstacleArrangement.NARROW_CORRIDOR:
            self.obstacle_paths = self.generate_narrow_corridor_obstacles(num_obstacles)

        for i, path in enumerate(self.obstacle_paths):
            obs_idx = i + 1  # avoid using 0 - that indicates free space in the grid
            for t, position in enumerate(path):
                # Reserve old & new position at this time step
                if t > 0:
                    self.reservation_matrix[path[t - 1].x, path[t - 1].y, t] = obs_idx
                self.reservation_matrix[position.x, position.y, t] = obs_idx

    """
    Generate dynamic obstacles that move around the grid. Initial positions and movements are random
    """
    def generate_dynamic_obstacles(self, obs_count: int) -> list[list[Position]]:
        obstacle_paths = []
        for _ in range(0, obs_count):
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
            diffs = [
                Position(0, 1),
                Position(0, -1),
                Position(1, 0),
                Position(-1, 0),
                Position(0, 0),
            ]

            for t in range(1, self.time_limit - 1):
                sampled_indices = np.random.choice(
                    len(diffs), size=5, replace=False, p=weights
                )
                rand_diffs = [diffs[i] for i in sampled_indices]

                valid_position = None
                for diff in rand_diffs:
                    new_position = positions[-1] + diff

                    if not self.valid_obstacle_position(new_position, t):
                        continue

                    valid_position = new_position
                    break

                # Impossible situation for obstacle - stay in place
                #   -> this can happen if the oaths of other obstacles this one
                if valid_position is None:
                    valid_position = positions[-1]

                positions.append(valid_position)

            obstacle_paths.append(positions)

        return obstacle_paths

    """
    Generate a line of obstacles in y at the center of the grid that move side-to-side in x
    Bottom half start moving right, top half start moving left. If `obs_count` is less than the length of
    the grid, only the first `obs_count` obstacles will be generated.
    """
    def obstacle_arrangement_1(self, obs_count: int) -> list[list[Position]]:
        obstacle_paths = []
        half_grid_x = self.grid_size[0] // 2
        half_grid_y = self.grid_size[1] // 2

        for y_idx in range(0, min(obs_count, self.grid_size[1])):
            moving_right = y_idx < half_grid_y
            position = Position(half_grid_x, y_idx)
            path = [position]

            for t in range(1, self.time_limit - 1):
                # sit in place every other time step
                if t % 2 == 0:
                    path.append(position)
                    continue

                # first check if we should switch direction (at edge of grid)
                if (moving_right and position.x == self.grid_size[0] - 1) or (
                    not moving_right and position.x == 0
                ):
                    moving_right = not moving_right
                # step in direction
                position = Position(
                    position.x + (1 if moving_right else -1), position.y
                )
                path.append(position)

            obstacle_paths.append(path)

        return obstacle_paths
    
    def generate_narrow_corridor_obstacles(self, obs_count: int) -> list[list[Position]]:
        obstacle_paths = []

        for y in range(0, self.grid_size[1]):
            if y > obs_count:
                break

            if y == self.grid_size[1] // 2:
                # Skip the middle row
                continue

            obstacle_path = []
            x = self.grid_size[0] // 2 # middle of the grid
            for t in range(0, self.time_limit - 1):
                obstacle_path.append(Position(x, y))

            obstacle_paths.append(obstacle_path)

        return obstacle_paths

    """
    Check if the given position is valid at time t

    input:
        position (Position): (x, y) position
        t (int): time step

    output:
        bool: True if position/time combination is valid, False otherwise
    """
    def valid_position(self, position: Position, t: int) -> bool:
        # Check if position is in grid
        if not self.inside_grid_bounds(position):
            return False

        # Check if position is not occupied at time t
        return self.reservation_matrix[position.x, position.y, t] == 0

    """
    Returns True if the given position is valid at time t and is not in the set of obstacle_avoid_points
    """
    def valid_obstacle_position(self, position: Position, t: int) -> bool:
        return (
            self.valid_position(position, t)
            and position not in self.obstacle_avoid_points
        )

    """
    Returns True if the given position is within the grid's boundaries
    """
    def inside_grid_bounds(self, position: Position) -> bool:
        return (
            position.x >= 0
            and position.x < self.grid_size[0]
            and position.y >= 0
            and position.y < self.grid_size[1]
        )

    """
    Sample a random position that is within the grid's boundaries

    output:
        Position: (x, y) position
    """
    def sample_random_position(self) -> Position:
        return Position(
            np.random.randint(0, self.grid_size[0]),
            np.random.randint(0, self.grid_size[1]),
        )

    """
    Returns a tuple of (x_positions, y_positions) of the obstacles at time t
    """
    def get_obstacle_positions_at_time(self, t: int) -> tuple[list[int], list[int]]:
        x_positions = []
        y_positions = []
        for obs_path in self.obstacle_paths:
            x_positions.append(obs_path[t].x)
            y_positions.append(obs_path[t].y)
        return (x_positions, y_positions)

    """
    Returns safe intervals for each cell.
    """
    def get_safe_intervals(self) -> np.ndarray:
        intervals = empty_2d_array_of_lists(self.grid_size[0], self.grid_size[1])
        for x in range(intervals.shape[0]):
            for y in range(intervals.shape[1]):
                intervals[x, y] = self.get_safe_intervals_at_cell(Position(x, y))

        return intervals

    """
    Generate the safe intervals for a given cell. The intervals will be in order of start time.
    ex: Interval (2, 3) will be before Interval (4, 5)
    """
    def get_safe_intervals_at_cell(self, cell: Position) -> list[Interval]:
        vals = self.reservation_matrix[cell.x, cell.y, :]
        # Find where the array is zero
        zero_mask = (vals == 0)

        # Identify transitions between zero and nonzero elements
        diff = np.diff(zero_mask.astype(int))

        # Start indices: where zeros begin (1 after a nonzero)
        start_indices = np.where(diff == 1)[0] + 1

        # End indices: where zeros stop (just before a nonzero)
        end_indices = np.where(diff == -1)[0]

        # Handle edge cases if the array starts or ends with zeros
        if zero_mask[0]:  # If the first element is zero, add index 0 to start_indices
            start_indices = np.insert(start_indices, 0, 0)
        if zero_mask[-1]:  # If the last element is zero, add the last index to end_indices
            end_indices = np.append(end_indices, len(vals) - 1)

        # Create pairs of (first zero, last zero)
        intervals = [Interval(int(start), int(end)) for start, end in zip(start_indices, end_indices)]

        # Remove intervals where a cell is only free for one time step. Those intervals not provide enough time to
        # move into and out of the cell each take 1 time step, and the cell is considered occupied during
        # both the time step when it is entering the cell,  and the time step when it is leaving the cell.
        intervals = [interval for interval in intervals if interval.start_time != interval.end_time]
        return intervals
    
    """
    Reserve an agent's path in the grid. Raises an exception if the agent's index is 0, or if a position is
    already reserved by a different agent.
    """
    def reserve_path(self, node_path: NodePath, agent_index: int):
        if agent_index == 0:
            raise Exception("Agent index cannot be 0")
        
        for i, node in enumerate(node_path.path):
            reservation_finish_time = node.time + 1
            if i < len(node_path.path) - 1:
                reservation_finish_time = node_path.path[i + 1].time

            self.reserve_position(node.position, agent_index, Interval(node.time, reservation_finish_time))

    """
    Reserve a position for the provided agent during the provided time interval.
    Raises an exception if the agent's index is 0, or if the position is already reserved by a different agent during the interval.
    """
    def reserve_position(self, position: Position, agent_index: int, interval: Interval):
        if agent_index == 0:
            raise Exception("Agent index cannot be 0")

        for t in range(interval.start_time, interval.end_time + 1):
            current_reserver = self.reservation_matrix[position.x, position.y, t]
            if current_reserver not in [0, agent_index]:
                raise Exception(
                    f"Agent {agent_index} tried to reserve a position already reserved by another agent: {position} at time {t}, reserved by {current_reserver}"
                )
            self.reservation_matrix[position.x, position.y, t] = agent_index

    """
    Clears the initial reservation for an agent by clearing reservations at its start position with its index for
    from time 0 to the time limit.
    """
    def clear_initial_reservation(self, position: Position, agent_index: int):
        for t in range(self.time_limit):
            if self.reservation_matrix[position.x, position.y, t] == agent_index:
                self.reservation_matrix[position.x, position.y, t] = 0

show_animation = True

def main():
    grid = Grid(
        np.array([11, 11]),
        num_obstacles=10,
        obstacle_arrangement=ObstacleArrangement.ARRANGEMENT1,
    )

    if not show_animation:
        return

    fig = plt.figure(figsize=(8, 7))
    ax = fig.add_subplot(
        autoscale_on=False,
        xlim=(0, grid.grid_size[0] - 1),
        ylim=(0, grid.grid_size[1] - 1),
    )
    ax.set_aspect("equal")
    ax.grid()
    ax.set_xticks(np.arange(0, 11, 1))
    ax.set_yticks(np.arange(0, 11, 1))
    (obs_points,) = ax.plot([], [], "ro", ms=15)

    # for stopping simulation with the esc key.
    plt.gcf().canvas.mpl_connect(
        "key_release_event", lambda event: [exit(0) if event.key == "escape" else None]
    )

    for i in range(0, grid.time_limit - 1):
        obs_positions = grid.get_obstacle_positions_at_time(i)
        obs_points.set_data(obs_positions[0], obs_positions[1])
        plt.pause(0.2)
    plt.show()


if __name__ == "__main__":
    main()
