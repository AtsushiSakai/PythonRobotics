"""
Dynamic Maze Solver

Dynamic BFS maze visualizer demonstrating breadth-first search on a grid.

author: Ujjansh Sundram

See Wikipedia: https://en.wikipedia.org/wiki/Breadth-first_search
"""

import matplotlib.pyplot as plt
import matplotlib.colors as mcolors
import numpy as np
from collections import deque
import random
import matplotlib.animation as animation


class MazeVisualizer:
    """
    Dynamic BFS maze-solving visualizer with moving target and evolving obstacles.
    """

    def __init__(self, maze, start, target):
        self.maze = np.array(maze, dtype=int)
        self.start_pos = start
        self.target_pos = target
        self.solver_pos = start

        self.rows, self.cols = self.maze.shape
        self.step_delay_ms = 200          # Animation frame delay
        self.target_move_interval = 5     # Target moves every N frames
        self.obstacle_change_prob = 0.01  # Random obstacle toggle probability

        # --- State Tracking ---
        self.path = []
        self.visited_nodes = set()
        self.breadcrumb_trail = [self.solver_pos]
        self.frame_count = 0

        # --- Plot Setup ---
        self.fig, self.ax = plt.subplots(figsize=(8, 6))
        plt.style.use('seaborn-v0_8-darkgrid')
        self.fig.patch.set_facecolor('#2c2c2c')
        self.ax.set_facecolor('#1e1e1e')

        self.ax.set_xticks([])
        self.ax.set_yticks([])

        # Base maze
        self.maze_plot = self.ax.imshow(self.maze, cmap='magma', interpolation='nearest')

        # Visited overlay
        self.visited_overlay = np.zeros((*self.maze.shape, 4))
        self.visited_plot = self.ax.imshow(self.visited_overlay, interpolation='nearest')

        # Path, breadcrumbs, solver, target
        self.path_line, = self.ax.plot([], [], 'g-', linewidth=3, alpha=0.7, label='Path')
        self.breadcrumbs_plot = self.ax.scatter([], [], c=[], cmap='viridis_r', s=50, alpha=0.6, label='Trail')
        self.solver_plot, = self.ax.plot(
            [self.solver_pos[1]], [self.solver_pos[0]],
            'o', markersize=15, color='#00ffdd', label='Solver'
        )
        self.target_plot, = self.ax.plot(
            [self.target_pos[1]], [self.target_pos[0]],
            '*', markersize=20, color='#ff006a', label='Target'
        )

        self.ax.legend(facecolor='gray', framealpha=0.5, loc='upper right')
        self.title = self.ax.set_title("Initializing Maze...", color='white', fontsize=14)

    def _bfs(self):
        """Performs BFS to find shortest path."""
        queue = deque([(self.solver_pos, [self.solver_pos])])
        visited = {self.solver_pos}

        while queue:
            (row, col), path = queue.popleft()

            if (row, col) == self.target_pos:
                return path, visited

            for d_row, d_col in [(-1, 0), (1, 0), (0, -1), (0, 1)]:
                n_row, n_col = row + d_row, col + d_col
                if 0 <= n_row < self.rows and 0 <= n_col < self.cols and \
                        self.maze[n_row][n_col] == 0 and (n_row, n_col) not in visited:
                    visited.add((n_row, n_col))
                    queue.append(((n_row, n_col), path + [(n_row, n_col)]))

        return None, visited

    def _update_target(self):
        """Moves the target randomly to an adjacent open cell."""
        t_row, t_col = self.target_pos
        moves = [(-1, 0), (1, 0), (0, -1), (0, 1)]
        random.shuffle(moves)
        for d_row, d_col in moves:
            n_row, n_col = t_row + d_row, t_col + d_col
            if 0 <= n_row < self.rows and 0 <= n_col < self.cols and self.maze[n_row][n_col] == 0:
                self.target_pos = (n_row, n_col)
                break

    def _update_obstacles(self):
        """Randomly toggle a few obstacles."""
        for row in range(self.rows):
            for col in range(self.cols):
                if (row, col) in [self.solver_pos, self.target_pos]:
                    continue
                if random.random() < self.obstacle_change_prob:
                    self.maze[row, col] = 1 - self.maze[row, col]

    def _update_frame(self, frame):
        """Main animation loop."""
        self.frame_count += 1

        # --- State ---
        if self.frame_count % self.target_move_interval == 0:
            self._update_target()
        self._update_obstacles()

        self.path, self.visited_nodes = self._bfs()

        # Move solver one step
        if self.path and len(self.path) > 1:
            self.solver_pos = self.path[1]
            self.breadcrumb_trail.append(self.solver_pos)

        # --- Visuals ---
        self.maze_plot.set_data(self.maze)

        # Visited overlay
        self.visited_overlay.fill(0)
        visited_color = mcolors.to_rgba('#0077b6', alpha=0.3)
        for row, col in self.visited_nodes:
            self.visited_overlay[row, col] = visited_color
        self.visited_plot.set_data(self.visited_overlay)

        # Path line
        if self.path:
            y, x = zip(*self.path)
            self.path_line.set_data(x, y)
        else:
            self.path_line.set_data([], [])

        # set_data() now receives sequences
        self.solver_plot.set_data([self.solver_pos[1]], [self.solver_pos[0]])
        self.target_plot.set_data([self.target_pos[1]], [self.target_pos[0]])

        # Breadcrumbs
        if self.breadcrumb_trail:
            y, x = zip(*self.breadcrumb_trail)
            colors = np.linspace(0.1, 1.0, len(y))
            self.breadcrumbs_plot.set_offsets(np.c_[x, y])
            self.breadcrumbs_plot.set_array(colors)

        # Title update
        if self.solver_pos == self.target_pos:
            self.title.set_text("Dynamic Maze Solver")
            self.title.set_color('lightgreen')
            self.anim.event_source.stop()
        else:
            path_len = len(self.path) if self.path else "N/A"
            self.title.set_text(f"Frame: {self.frame_count} | Path Length: {path_len}")
            self.title.set_color('white' if self.path else 'coral')

        return [
            self.maze_plot, self.visited_plot, self.path_line,
            self.solver_plot, self.target_plot, self.breadcrumbs_plot, self.title
        ]

    def run(self):
        """Starts the animation."""
        self.anim = animation.FuncAnimation(
            self.fig,
            self._update_frame,
            frames=500,
            interval=self.step_delay_ms,
            blit=False,  
            repeat=False
        )
        plt.show()


if __name__ == "__main__":
    initial_maze = [
        [0, 1, 0, 0, 0, 0, 0, 0, 1, 0],
        [0, 1, 0, 1, 1, 0, 1, 0, 1, 0],
        [0, 0, 0, 1, 0, 0, 1, 0, 0, 0],
        [0, 1, 0, 1, 0, 1, 1, 1, 1, 0],
        [0, 1, 0, 0, 0, 0, 0, 0, 1, 0],
        [0, 1, 1, 1, 1, 1, 1, 0, 1, 0],
        [0, 0, 0, 0, 0, 0, 1, 0, 0, 0],
        [1, 1, 1, 1, 0, 1, 1, 1, 1, 0],
        [0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
    ]

    start_point = (0, 0)
    end_point = (8, 9)

    visualizer = MazeVisualizer(initial_maze, start_point, end_point)
    visualizer.run()
