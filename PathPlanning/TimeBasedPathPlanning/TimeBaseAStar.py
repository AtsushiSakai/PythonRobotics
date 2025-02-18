import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from moving_obstacles import Grid, Position
import heapq
from typing import Generator
import random

# Seed randomness for reproducibility
RANDOM_SEED = 42
random.seed(RANDOM_SEED)
np.random.seed(RANDOM_SEED)

class Node:
    position: Position
    time: int
    heuristic: int
    parent_index: int

    def __init__(self, position: Position, time: int, heuristic: int, parent_index: int):
        self.position = position
        self.time = time
        self.heuristic = heuristic
        self.parent_index = parent_index
        
    def __lt__(self, other):
        return (self.time + self.heuristic) < (other.time + other.heuristic)

    def __repr__(self):
        return f"Node(position={self.position}, time={self.time}, heuristic={self.heuristic}, parent_index={self.parent_index})"

class NodePath:
    path: list[Node]

    def __init__(self, path: list[Node]):
        self.path = path
    
    def get_position(self, time: int) -> Position:
        # TODO: this is inefficient
        for i in range(0, len(self.path) - 2):
            if self.path[i + 1].time > time:
                print(f"position @ {i} is {self.path[i].position}")
                return self.path[i].position
        
        if len(self.path) > 0:
            return self.path[-1].position

        return None
    
    def goal_reached_time(self) -> int:
        return self.path[-1].time
    
    def __repr__(self):
        repr_string = ""
        for (i, node) in enumerate(self.path):
            repr_string += f"{i}: {node}\n"
        return repr_string

class TimeBasedAStar:
    grid: Grid
    start: Position
    goal: Position

    def __init__(self, grid: Grid, start: Position, goal: Position):
        self.grid = grid
        self.start = start
        self.goal = goal

    def plan(self, verbose: bool = False) -> NodePath:
        open_set = []
        heapq.heappush(open_set, Node(self.start, 0, self.calculate_heuristic(self.start), -1))

        # TODO: is vec good here?
        expanded_set = []
        while open_set:
            expanded_node: Node = heapq.heappop(open_set)
            if verbose:
                print("Expanded node:", expanded_node)

            if expanded_node.time + 1 >= self.grid.time_limit:
                if verbose:
                    print(f"\tSkipping node that is past time limit: {expanded_node}")
                continue

            if expanded_node.position == self.goal:
                print(f"Found path to goal after {len(expanded_set)} expansions")
                path = []
                path_walker: Node = expanded_node
                while path_walker.parent_index != -1:
                    path.append(path_walker)
                    path_walker = expanded_set[path_walker.parent_index]
                # TODO: fix hack around bad while condiiotn
                path.append(path_walker)
                
                # reverse path so it goes start -> goal
                path.reverse()
                return NodePath(path)

            expanded_idx = len(expanded_set)
            expanded_set.append(expanded_node)

            for child in self.generate_successors(expanded_node, expanded_idx, verbose):
                heapq.heappush(open_set, child)
        
        raise Exception("No path found")
    
    def generate_successors(self, parent_node: Node, parent_node_idx: int, verbose: bool) -> Generator[Node, None, None]:
        diffs = [Position(0, 1), Position(0, -1), Position(1, 0), Position(-1, 0), Position(0, 0)]
        for diff in diffs:
            new_pos = parent_node.position + diff
            if self.grid.valid_position(new_pos, parent_node.time+1):
                new_node = Node(new_pos, parent_node.time+1, self.calculate_heuristic(new_pos), parent_node_idx)
                if verbose:
                    print("\tNew successor node: ", new_node)
                yield new_node

    def calculate_heuristic(self, position) -> int:
        diff = self.goal - position
        return abs(diff.x) + abs(diff.y)

show_animation = True
def main():
    start = Position(1, 1)
    goal = Position(19, 19)
    grid_side_length = 21
    grid = Grid(np.array([grid_side_length, grid_side_length]), num_obstacles=115, obstacle_avoid_points=[start, goal])

    planner = TimeBasedAStar(grid, start, goal)
    verbose = False
    path = planner.plan(verbose)

    if verbose:
        print(f"Path: {path}")

    if not show_animation:
        return

    fig = plt.figure(figsize=(10, 7))
    ax = fig.add_subplot(autoscale_on=False, xlim=(0, grid.grid_size[0]-1), ylim=(0, grid.grid_size[1]-1))
    ax.set_aspect('equal')
    ax.grid()
    ax.set_xticks(np.arange(0, grid_side_length, 1))
    ax.set_yticks(np.arange(0, grid_side_length, 1))

    start_and_goal, = ax.plot([], [], 'mD', ms=15, label="Start and Goal") 
    start_and_goal.set_data([start.x, goal.x], [start.y, goal.y])
    obs_points, = ax.plot([], [], 'ro', ms=15, label="Obstacles")
    path_points, = ax.plot([], [], 'bo', ms=10, label="Path Found")
    ax.legend(bbox_to_anchor=(1.05, 1))

    def get_frame(i):
        obs_x_points = []
        obs_y_points = []
        for obs_path in grid.obstacle_paths:
            obs_pos = obs_path[i]
            obs_x_points.append(obs_pos.x)
            obs_y_points.append(obs_pos.y)
        obs_points.set_data(obs_x_points, obs_y_points)

        path_position = path.get_position(i)
        path_points.set_data([path_position.x], [path_position.y])
        return start_and_goal, obs_points, path_points

    _ani = animation.FuncAnimation(
        fig, get_frame, path.goal_reached_time(), interval=500, blit=True, repeat=False)
    plt.show()

if __name__ == '__main__':
    main()