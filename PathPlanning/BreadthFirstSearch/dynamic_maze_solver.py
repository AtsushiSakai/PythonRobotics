import matplotlib.pyplot as plt
import matplotlib.colors as mcolors
import numpy as np
from collections import deque
import random
import matplotlib.animation as animation

class MazeVisualizer:
    """
    A class to create a beautiful and interesting visualization
    of a dynamic maze-solving algorithm (BFS).
    """

    def __init__(self, maze, start, target):
        self.maze = np.array(maze)
        self.start_pos = start
        self.target_pos = target
        self.solver_pos = start
        
        self.rows, self.cols = self.maze.shape
        
        # --- Configurable Parameters ---
        self.step_delay_ms = 200  # Animation frame delay in milliseconds
        self.target_move_interval = 5  # Target moves every N frames
        self.obstacle_change_prob = 0.01 # Probability of a wall changing

        # --- State Tracking ---
        self.path = []
        self.visited_nodes = set()
        self.breadcrumb_trail = [self.solver_pos]
        self.frame_count = 0

        # --- Plotting Setup ---
        self.fig, self.ax = plt.subplots(figsize=(8, 6))
        plt.style.use('seaborn-v0_8-darkgrid')
        self.fig.patch.set_facecolor('#2c2c2c')
        self.ax.set_facecolor('#1e1e1e')
        
        # Hide axes ticks and labels for a cleaner look
        self.ax.set_xticks([])
        self.ax.set_yticks([])
        
        # Maze plot
        self.maze_plot = self.ax.imshow(self.maze, cmap='magma', interpolation='nearest')
        
        # Visited nodes plot (semi-transparent overlay)
        self.visited_overlay = np.zeros((*self.maze.shape, 4)) # RGBA
        self.visited_plot = self.ax.imshow(self.visited_overlay, interpolation='nearest')

        # Path, solver, target, and breadcrumbs plots
        self.path_line, = self.ax.plot([], [], 'g-', linewidth=3, alpha=0.7, label='Path')
        self.breadcrumbs_plot = self.ax.scatter([], [], c=[], cmap='viridis_r', s=50, alpha=0.6, label='Trail')
        self.solver_plot, = self.ax.plot(self.solver_pos[1], self.solver_pos[0], 'o', markersize=15, color='#00ffdd', label='Solver')
        self.target_plot, = self.ax.plot(self.target_pos[1], self.target_pos[0], '*', markersize=20, color='#ff006a', label='Target')
        
        self.ax.legend(facecolor='gray', framealpha=0.5, loc='upper right')
        self.title = self.ax.set_title("Initializing Maze...", color='white', fontsize=14)

    def _bfs(self):
        """Performs BFS to find the shortest path and returns path and visited nodes."""
        queue = deque([(self.solver_pos, [self.solver_pos])])
        visited = {self.solver_pos}

        while queue:
            (r, c), path = queue.popleft()

            if (r, c) == self.target_pos:
                return path, visited

            for dr, dc in [(-1, 0), (1, 0), (0, -1), (0, 1)]:
                nr, nc = r + dr, c + dc
                if 0 <= nr < self.rows and 0 <= nc < self.cols and \
                   self.maze[nr][nc] == 0 and (nr, nc) not in visited:
                    visited.add((nr, nc))
                    new_path = list(path)
                    new_path.append((nr, nc))
                    queue.append(((nr, nc), new_path))
        
        return None, visited # No path found

    def _update_target(self):
        """Moves the target to a random adjacent valid cell."""
        tr, tc = self.target_pos
        moves = [(-1, 0), (1, 0), (0, -1), (0, 1)]
        random.shuffle(moves)
        for dr, dc in moves:
            nr, nc = tr + dr, tc + dc
            if 0 <= nr < self.rows and 0 <= nc < self.cols and self.maze[nr][nc] == 0:
                self.target_pos = (nr, nc)
                break

    def _update_obstacles(self):
        """Randomly toggles a few obstacle cells."""
        for r in range(self.rows):
            for c in range(self.cols):
                # Avoid changing start/target positions
                if (r,c) == self.solver_pos or (r,c) == self.target_pos:
                    continue
                if random.random() < self.obstacle_change_prob:
                    self.maze[r, c] = 1 - self.maze[r, c] # Toggle 0 to 1 or 1 to 0

    def _update_frame(self, frame):
        """Main animation loop function."""
        self.frame_count += 1
        
        # --- Update Game State ---
        if self.frame_count % self.target_move_interval == 0:
            self._update_target()
        
        self._update_obstacles()
        
        self.path, self.visited_nodes = self._bfs()

        if self.path and len(self.path) > 1:
            self.solver_pos = self.path[1] # Move solver one step
            self.breadcrumb_trail.append(self.solver_pos)

        # --- Update Visuals ---
        # Update maze and visited nodes overlay
        self.maze_plot.set_data(self.maze)
        self.visited_overlay.fill(0) # Reset overlay
        visited_color = mcolors.to_rgba('#0077b6', alpha=0.3)
        for r, c in self.visited_nodes:
            self.visited_overlay[r, c] = visited_color
        self.visited_plot.set_data(self.visited_overlay)
        
        # Update path line
        if self.path:
            path_y, path_x = zip(*self.path)
            self.path_line.set_data(path_x, path_y)
        else:
            self.path_line.set_data([], [])

        # Update solver and target positions
        self.solver_plot.set_data(self.solver_pos[1], self.solver_pos[0])
        self.target_plot.set_data(self.target_pos[1], self.target_pos[0])

        # Update breadcrumbs
        if self.breadcrumb_trail:
            trail_y, trail_x = zip(*self.breadcrumb_trail)
            colors = np.linspace(0.1, 1.0, len(trail_y))
            self.breadcrumbs_plot.set_offsets(np.c_[trail_x, trail_y])
            self.breadcrumbs_plot.set_array(colors)

        # Update title and check for win condition
        if self.solver_pos == self.target_pos:
            self.title.set_text("Target Reached! 🎉")
            self.title.set_color('lightgreen')
            self.anim.event_source.stop() # Stop animation
        else:
            path_len_str = len(self.path) if self.path else "N/A"
            self.title.set_text(f"Frame: {self.frame_count} | Path Length: {path_len_str}")
            if not self.path:
                self.title.set_color('coral')
            else:
                self.title.set_color('white')
        
        return [self.maze_plot, self.visited_plot, self.path_line, self.solver_plot, 
                self.target_plot, self.breadcrumbs_plot, self.title]

    def run(self):
        """Starts the animation."""
        self.anim = animation.FuncAnimation(
            self.fig, 
            self._update_frame, 
            frames=200, # Can be increased for longer animation
            interval=self.step_delay_ms, 
            blit=True, 
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

    visualizer = MazeVisualizer(maze=initial_maze, start=start_point, target=end_point)
    visualizer.run()