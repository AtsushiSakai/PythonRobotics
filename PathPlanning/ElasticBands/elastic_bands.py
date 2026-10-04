"""
Elastic Bands

author: Wang Zheng (@Aglargil)

Reference:

- [Elastic Bands: Connecting Path Planning and Control]
(http://www8.cs.umu.se/research/ifor/dl/Control/elastic%20bands.pdf)
"""

import numpy as np
import sys
import pathlib
import matplotlib.pyplot as plt
from matplotlib.patches import Circle

sys.path.append(str(pathlib.Path(__file__).parent.parent.parent))

from Mapping.DistanceMap.distance_map import compute_sdf_scipy

# Elastic Bands Params
MAX_BUBBLE_RADIUS = 100
MIN_BUBBLE_RADIUS = 10
RHO0 = 20.0  # Maximum distance for applying repulsive force
KC = 0.05  # Contraction force gain
KR = -0.1  # Repulsive force gain
LAMBDA = 0.7  # Overlap constraint factor
STEP_SIZE = 3.0  # Step size for calculating gradient

# Visualization Params
ENABLE_PLOT = True
# ENABLE_INTERACTIVE is True allows user to add obstacles by left clicking
# and add path points by right clicking and start planning by middle clicking
ENABLE_INTERACTIVE = False
# ENABLE_SAVE_DATA is True allows saving the path and obstacles which added
# by user in interactive mode to file
ENABLE_SAVE_DATA = False
MAX_ITER = 50


class Bubble:
    def __init__(self, position, radius):
        self.pos = np.array(position)  # Bubble center coordinates [x, y]
        self.radius = radius  # Safety distance radius ρ(b)
        if self.radius > MAX_BUBBLE_RADIUS:
            self.radius = MAX_BUBBLE_RADIUS
        if self.radius < MIN_BUBBLE_RADIUS:
            self.radius = MIN_BUBBLE_RADIUS


class ElasticBands:
    def __init__(
        self,
        initial_path,
        obstacles,
        rho0=RHO0,
        kc=KC,
        kr=KR,
        lambda_=LAMBDA,
        step_size=STEP_SIZE,
    ):
        self.distance_map = compute_sdf_scipy(obstacles)
        self.bubbles = [
            Bubble(p, self.compute_rho(p)) for p in initial_path
        ]  # Initialize bubble chain
        self.kc = kc  # Contraction force gain
        self.kr = kr  # Repulsive force gain
        self.rho0 = rho0  # Maximum distance for applying repulsive force
        self.lambda_ = lambda_  # Overlap constraint factor
        self.step_size = step_size  # Step size for calculating gradient
        self._maintain_overlap()

    def compute_rho(self, position):
        """Compute the distance field value at the position"""
        return self.distance_map[int(position[0]), int(position[1])]

    def contraction_force(self, i):
        """Calculate internal contraction force for the i-th bubble"""
        if i == 0 or i == len(self.bubbles) - 1:
            return np.zeros(2)

        prev = self.bubbles[i - 1].pos
        next_ = self.bubbles[i + 1].pos
        current = self.bubbles[i].pos

        # f_c = kc * ( (prev-current)/|prev-current| + (next-current)/|next-current| )
        dir_prev = (prev - current) / (np.linalg.norm(prev - current) + 1e-6)
        dir_next = (next_ - current) / (np.linalg.norm(next_ - current) + 1e-6)
        return self.kc * (dir_prev + dir_next)

    def repulsive_force(self, i):
        """Calculate external repulsive force for the i-th bubble"""
        h = self.step_size  # Step size
        b = self.bubbles[i].pos
        rho = self.bubbles[i].radius

        if rho >= self.rho0:
            return np.zeros(2)

        # Finite difference approximation of the gradient ∂ρ/∂b
        dx = np.array([h, 0])
        dy = np.array([0, h])
        grad_x = (self.compute_rho(b - dx) - self.compute_rho(b + dx)) / (2 * h)
        grad_y = (self.compute_rho(b - dy) - self.compute_rho(b + dy)) / (2 * h)
        grad = np.array([grad_x, grad_y])

        return self.kr * (self.rho0 - rho) * grad

    def update_bubbles(self):
        """Update bubble positions"""
        new_bubbles = []
        for i in range(len(self.bubbles)):
            if i == 0 or i == len(self.bubbles) - 1:
                new_bubbles.append(self.bubbles[i])  # Fixed start and end points
                continue

            f_total = self.contraction_force(i) + self.repulsive_force(i)
            v = self.bubbles[i - 1].pos - self.bubbles[i + 1].pos

            # Remove tangential component
            f_star = f_total - f_total * v * v / (np.linalg.norm(v) ** 2 + 1e-6)

            alpha = self.bubbles[i].radius  # Adaptive step size
            new_pos = self.bubbles[i].pos + alpha * f_star
            new_pos = np.clip(new_pos, 0, 499)
            new_radius = self.compute_rho(new_pos)

            # Update bubble and maintain overlap constraint
            new_bubble = Bubble(new_pos, new_radius)
            new_bubbles.append(new_bubble)

        self.bubbles = new_bubbles
        self._maintain_overlap()

    def _maintain_overlap(self):
        """Maintain bubble chain continuity (simplified insertion/deletion mechanism)"""
        # Insert bubbles
        i = 0
        while i < len(self.bubbles) - 1:
            bi, bj = self.bubbles[i], self.bubbles[i + 1]
            dist = np.linalg.norm(bi.pos - bj.pos)
            if dist > self.lambda_ * (bi.radius + bj.radius):
                new_pos = (bi.pos + bj.pos) / 2
                rho = self.compute_rho(
                    new_pos
                )  # Calculate new radius using environment model
                self.bubbles.insert(i + 1, Bubble(new_pos, rho))
                i += 2  # Skip the processed region
            else:
                i += 1

        # Delete redundant bubbles
        i = 1
        while i < len(self.bubbles) - 1:
            prev = self.bubbles[i - 1]
            next_ = self.bubbles[i + 1]
            dist = np.linalg.norm(prev.pos - next_.pos)
            if dist <= self.lambda_ * (prev.radius + next_.radius):
                del self.bubbles[i]  # Delete if redundant
            else:
                i += 1


class ElasticBandsVisualizer:
    def __init__(self):
        self.obstacles = np.zeros((500, 500))
        self.obstacles_points = []
        self.path_points = []
        self.elastic_band = None
        self.running = True

        if ENABLE_PLOT:
            self.fig, self.ax = plt.subplots(figsize=(8, 8))
            self.fig.canvas.mpl_connect("close_event", self.on_close)
            self.ax.set_xlim(0, 500)
            self.ax.set_ylim(0, 500)

        if ENABLE_INTERACTIVE:
            self.path_points = []  # Add a list to store path points
            # Connect mouse events
            self.fig.canvas.mpl_connect("button_press_event", self.on_click)
        else:
            self.path_points = np.load(pathlib.Path(__file__).parent / "path.npy")
            self.obstacles_points = np.load(
                pathlib.Path(__file__).parent / "obstacles.npy"
            )
            for x, y in self.obstacles_points:
                self.add_obstacle(x, y)
            self.plan_path()

        self.plot_background()

    def on_close(self, event):
        """Handle window close event"""
        self.running = False
        plt.close("all")  # Close all figure windows

    def plot_background(self):
        """Plot the background grid"""
        if not ENABLE_PLOT or not self.running:
            return

        self.ax.cla()
        self.ax.set_xlim(0, 500)
        self.ax.set_ylim(0, 500)
        self.ax.grid(True)

        if ENABLE_INTERACTIVE:
            self.ax.set_title(
                "Elastic Bands Path Planning\n"
                "Left click: Add obstacles\n"
                "Right click: Add path points\n"
                "Middle click: Start planning",
                pad=20,
            )
        else:
            self.ax.set_title("Elastic Bands Path Planning", pad=20)

        if self.path_points:
            self.ax.plot(
                [p[0] for p in self.path_points],
                [p[1] for p in self.path_points],
                "yo",
                markersize=8,
            )

        self.ax.imshow(self.obstacles.T, origin="lower", cmap="binary", alpha=0.8)
        self.ax.plot([], [], color="black", label="obstacles")
        if self.elastic_band is not None:
            path = [b.pos.tolist() for b in self.elastic_band.bubbles]
            path = np.array(path)
            self.ax.plot(path[:, 0], path[:, 1], "b-", linewidth=2, label="path")

            for bubble in self.elastic_band.bubbles:
                circle = Circle(
                    bubble.pos, bubble.radius, fill=False, color="g", alpha=0.3
                )
                self.ax.add_patch(circle)
                self.ax.plot(bubble.pos[0], bubble.pos[1], "bo", markersize=10)
            self.ax.plot([], [], color="green", label="bubbles")

        self.ax.legend(loc="upper right")
        plt.draw()
        plt.pause(0.01)

    def add_obstacle(self, x, y):
        """Add an obstacle at the given coordinates"""
        size = 30  # Side length of the square
        half_size = size // 2
        x_start = max(0, x - half_size)
        x_end = min(self.obstacles.shape[0], x + half_size)
        y_start = max(0, y - half_size)
        y_end = min(self.obstacles.shape[1], y + half_size)
        self.obstacles[x_start:x_end, y_start:y_end] = 1

    def on_click(self, event):
        """Handle mouse click events"""
        if event.inaxes != self.ax:
            return

        x, y = int(event.xdata), int(event.ydata)

        if event.button == 1:  # Left click to add obstacles
            self.add_obstacle(x, y)
            self.obstacles_points.append([x, y])

        elif event.button == 3:  # Right click to add path points
            self.path_points.append([x, y])

        elif event.button == 2:  # Middle click to end path input and start planning
            if len(self.path_points) >= 2:
                if ENABLE_SAVE_DATA:
                    np.save(
                        pathlib.Path(__file__).parent / "path.npy", self.path_points
                    )
                    np.save(
                        pathlib.Path(__file__).parent / "obstacles.npy",
                        self.obstacles_points,
                    )
                self.plan_path()

        self.plot_background()

    def plan_path(self):
        """Plan the path"""

        initial_path = self.path_points
        # Create an elastic band object and optimize
        self.elastic_band = ElasticBands(initial_path, self.obstacles)
        for _ in range(MAX_ITER):
            self.elastic_band.update_bubbles()
            self.path_points = [b.pos for b in self.elastic_band.bubbles]
            self.plot_background()


if __name__ == "__main__":
    _ = ElasticBandsVisualizer()
    if ENABLE_PLOT:
        plt.show(block=True)
