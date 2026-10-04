"""
Particle Swarm Optimization (PSO) Path Planning

author: Anish (@anishk85)

See Wikipedia article (https://en.wikipedia.org/wiki/Particle_swarm_optimization)

References:
    - Kennedy, J.; Eberhart, R. (1995). "Particle Swarm Optimization"
    - Shi, Y.; Eberhart, R. (1998). "A Modified Particle Swarm Optimizer"
    - https://machinelearningmastery.com/a-gentle-introduction-to-particle-swarm-optimization/

This implementation uses PSO to find collision-free paths by treating
path planning as an optimization problem where particles explore the
search space to minimize distance to target while avoiding obstacles.
"""
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import matplotlib.patches as patches
import signal
import sys
# Add show_animation flag for consistency with other planners
show_animation = True

def signal_handler(sig, frame):
    print("\nExiting...")
    plt.close("all")
    sys.exit(0)

signal.signal(signal.SIGINT, signal_handler)

class Particle:
    """Represents a single particle in the PSO swarm.
    Each particle maintains its current position, velocity, and personal best
    position discovered during the search. Particles explore the search space
    by updating their velocity based on personal experience (cognitive component)
    and swarm knowledge (social component).
    Attributes:
        search_bounds: List of tuples [(x_min, x_max), (y_min, y_max)] defining search space
        max_velocity: Maximum velocity allowed in each dimension (5% of search space range)
        position: Current 2D position [x, y] in search space
        velocity: Current velocity vector [vx, vy]
        personal_best_position: Personal best position found so far
        personal_best_value: Fitness value at personal best position
        path: List of all positions visited by this particle
    """

    def __init__(self, search_bounds, spawn_bounds):
        self.search_bounds = search_bounds
        self.max_velocity = np.array([(b[1] - b[0]) * 0.05 for b in search_bounds])
        self.position = np.array([np.random.uniform(b[0], b[1]) for b in spawn_bounds])
        self.velocity = np.random.randn(2) * 0.1
        self.personal_best_position = self.position.copy()
        self.personal_best_value = np.inf
        self.path = [self.position.copy()]

    def update_velocity(self, gbest_pos, w, c1, c2):
        """Update particle velocity using PSO equation:
        v = w*v + c1*r1*(personal_best - x) + c2*r2*(gbest - x)
        """
        r1 = np.random.rand(2)
        r2 = np.random.rand(2)
        cognitive = c1 * r1 * (self.personal_best_position - self.position)
        social = c2 * r2 * (gbest_pos - self.position)
        self.velocity = w * self.velocity + cognitive + social
        self.velocity = np.clip(self.velocity, -self.max_velocity, self.max_velocity)

    def update_position(self):
        self.position = self.position + self.velocity
        # Keep in bounds
        for i in range(2):
            self.position[i] = np.clip(
                self.position[i], self.search_bounds[i][0], self.search_bounds[i][1]
            )
        self.path.append(self.position.copy())

class PSOSwarm:

    def __init__(
        self, n_particles, max_iter, target, search_bounds, spawn_bounds, obstacles
    ):
        self.n_particles = n_particles
        self.max_iter = max_iter
        self.target = np.array(target)
        self.obstacles = obstacles
        self.search_bounds = search_bounds
        # PSO parameters
        self.w_start = 0.9  # Initial inertia weight
        self.w_end = 0.4  # Final inertia weight
        self.c1 = 1.5  # Cognitive coefficient
        self.c2 = 1.5  # Social coefficient
        # Initialize particles
        self.particles = [
            Particle(search_bounds, spawn_bounds) for _ in range(n_particles)
        ]
        self.gbest_position = None
        self.gbest_value = np.inf
        self.gbest_path = []
        self.iteration = 0

    def fitness(self, pos):
        """Calculate fitness - distance to target + obstacle penalty"""
        dist = np.linalg.norm(pos - self.target)
        # Obstacle penalty
        penalty = 0
        for ox, oy, r in self.obstacles:
            obs_dist = np.linalg.norm(pos - np.array([ox, oy]))
            if obs_dist < r:
                penalty += 1000  # Inside obstacle
            elif obs_dist < r + 5:
                penalty += 50 / (obs_dist - r + 0.1)  # Too close
        return dist + penalty

    def check_collision(self, start, end, obstacle):
        """Check if path from start to end hits obstacle using line-circle intersection
            Args:
            start: Starting position (numpy array)
            end: Ending position (numpy array)
            obstacle: Tuple (ox, oy, r) representing obstacle center and radius
        Returns:
            bool: True if collision detected, False otherwise
        """
        ox, oy, r = obstacle
        center = np.array([ox, oy])
        # Vector math for line-circle intersection
        d = end - start
        f = start - center
        a = np.dot(d, d)
        # Guard against zero-length steps to prevent ZeroDivisionError
        if a < 1e-10:  # Near-zero length step
            # Check if start point is inside obstacle
            return np.linalg.norm(f) <= r
        b = 2 * np.dot(f, d)
        c = np.dot(f, f) - r * r
        discriminant = b * b - 4 * a * c
        if discriminant < 0:
            return False
        # Check if intersection on segment
        sqrt_discriminant = np.sqrt(discriminant)
        t1 = (-b - sqrt_discriminant) / (2 * a)
        t2 = (-b + sqrt_discriminant) / (2 * a)
        return (0 <= t1 <= 1) or (0 <= t2 <= 1)

    def step(self):
        """Run one PSO iteration
        Returns:
            bool: True if algorithm should continue, False if completed
        """
        if self.iteration >= self.max_iter:
            return False
        # Update inertia weight (linear decay)
        w = self.w_start - (self.w_start - self.w_end) * (
            self.iteration / self.max_iter
        )
        # Evaluate all particles
        for particle in self.particles:
            value = self.fitness(particle.position)
            # Update personal best
            if value < particle.personal_best_value:
                particle.personal_best_value = value
                particle.personal_best_position = particle.position.copy()
            # Update global best
            if value < self.gbest_value:
                self.gbest_value = value
                self.gbest_position = particle.position.copy()
        if self.gbest_position is not None:
            self.gbest_path.append(self.gbest_position.copy())
        # Update particles
        for particle in self.particles:
            particle.update_velocity(self.gbest_position, w, self.c1, self.c2)
            # Predict next position
            next_pos = particle.position + particle.velocity
            # Check collision
            collision = False
            for obs in self.obstacles:
                if self.check_collision(particle.position, next_pos, obs):
                    collision = True
                    break
            if collision:
                # Reduce velocity if collision detected
                particle.velocity *= 0.2
            particle.update_position()
        self.iteration += 1
        if show_animation and self.iteration % 20 == 0:
            print(
                f"Iteration {self.iteration}/{self.max_iter}, Best: {self.gbest_value:.2f}"
            )
        return True

def main():
    """Run PSO path planning algorithm demonstration.
    This function demonstrates PSO-based path planning with the following setup:
    - 15 particles exploring a (-50,50) x (-50,50) search space
    - Start zone: (-45,-45) to (-35,-35)
    - Target: (40, 35)
    - 4 circular obstacles with collision avoidance
    - Real-time visualization showing particle convergence (if show_animation=True)
    - Headless mode support for testing (if show_animation=False)
    The algorithm runs for up to 150 iterations, displaying particle movement,
    personal/global best positions, and the evolving optimal path.
    """
    print(__file__ + " start!!")
    # Set matplotlib backend for headless environments
    if not show_animation:
        plt.switch_backend("Agg")  # Use non-GUI backend for testing
    # Setup parameters
    N_PARTICLES = 15
    MAX_ITER = 150
    SEARCH_BOUNDS = [(-50, 50), (-50, 50)]
    TARGET = [40, 35]
    SPAWN_AREA = [(-45, -35), (-45, -35)]
    OBSTACLES = [(10, 15, 8), (-20, 0, 12), (20, -25, 10), (-5, -30, 7)]
    swarm = PSOSwarm(
        n_particles=N_PARTICLES,
        max_iter=MAX_ITER,
        target=TARGET,
        search_bounds=SEARCH_BOUNDS,
        spawn_bounds=SPAWN_AREA,
        obstacles=OBSTACLES,
    )
    # pragma: no cover
    if show_animation:
        # Visualization setup
        signal.signal(signal.SIGINT, signal_handler)
        fig, ax = plt.subplots(figsize=(10, 10))
        ax.set_xlim(SEARCH_BOUNDS[0])
        ax.set_ylim(SEARCH_BOUNDS[1])
        ax.set_title("PSO Path Planning with Collision Avoidance", fontsize=14)
        ax.grid(True, alpha=0.3)
        # Draw obstacles
        for ox, oy, r in OBSTACLES:
            circle = patches.Circle((ox, oy), r, color="gray", alpha=0.7)
            ax.add_patch(circle)
        # Draw spawn area
        spawn_rect = patches.Rectangle(
            (SPAWN_AREA[0][0], SPAWN_AREA[1][0]),
            SPAWN_AREA[0][1] - SPAWN_AREA[0][0],
            SPAWN_AREA[1][1] - SPAWN_AREA[1][0],
            linewidth=2,
            edgecolor="green",
            facecolor="green",
            alpha=0.2,
            label="Start Zone",
        )
        ax.add_patch(spawn_rect)
        # Draw target
        ax.plot(TARGET[0], TARGET[1], "r*", markersize=20, label="Target")
        # Initialize plot elements
        particles_scatter = ax.scatter(
            [], [], c="blue", s=50, alpha=0.6, label="Particles"
        )
        gbest_scatter = ax.plot([], [], "yo", markersize=12, label="Best Position")[0]
        particle_paths = [
            ax.plot([], [], "b-", lw=0.5, alpha=0.2)[0] for _ in range(N_PARTICLES)
        ]
        gbest_path_line = ax.plot([], [], "y--", lw=2, alpha=0.8, label="Best Path")[0]
        iteration_text = ax.text(
            0.02,
            0.95,
            "",
            transform=ax.transAxes,
            fontsize=12,
            verticalalignment="top",
            bbox=dict(boxstyle="round", facecolor="wheat", alpha=0.5),
        )
        ax.legend(loc="upper right")
        def animate(frame):
            """Animation function for matplotlib FuncAnimation"""
            if not swarm.step():
                return (
                    particles_scatter,
                    gbest_scatter,
                    gbest_path_line,
                    iteration_text,
                    *particle_paths,
                )
            # Update particle positions
            positions = np.array([p.position for p in swarm.particles])
            particles_scatter.set_offsets(positions)
            # Update particle paths
            for i, particle in enumerate(swarm.particles):
                if len(particle.path) > 1:
                    path = np.array(particle.path)
                    particle_paths[i].set_data(path[:, 0], path[:, 1])
            # Update global best
            if swarm.gbest_position is not None:
                gbest_scatter.set_data(
                    [swarm.gbest_position[0]], [swarm.gbest_position[1]]
                )
                if len(swarm.gbest_path) > 1:
                    gbest = np.array(swarm.gbest_path)
                    gbest_path_line.set_data(gbest[:, 0], gbest[:, 1])
            # Update text
            iteration_text.set_text(
                f"Iteration: {swarm.iteration}/{MAX_ITER}\n"
                f"Best Fitness: {swarm.gbest_value:.2f}"
            )
            return (
                particles_scatter,
                gbest_scatter,
                gbest_path_line,
                iteration_text,
                *particle_paths,
            )
        # Create animation and store reference to prevent garbage collection
        animation_ref = animation.FuncAnimation(
            fig, animate, frames=MAX_ITER, interval=100, blit=True, repeat=False
        )
        plt.tight_layout()
        plt.show()
        # Keep reference to prevent garbage collection
        return animation_ref
    else:
        # Run without animation for testing
        print("Running PSO algorithm without animation...")
        iteration_count = 0
        while swarm.step():
            iteration_count += 1
            if iteration_count >= MAX_ITER:
                break
        print(f"PSO completed after {iteration_count} iterations")
        print(f"Best fitness: {swarm.gbest_value:.2f}")
        if swarm.gbest_position is not None:
            print(
                f"Best position: [{swarm.gbest_position[0]:.2f}, {swarm.gbest_position[1]:.2f}]"
            )
        return None
if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\nProgram interrupted by user")
        plt.close("all")
        sys.exit(0)