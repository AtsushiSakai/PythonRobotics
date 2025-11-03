"""
Genetic Algorithm (GA) Path Planning

author: Dennis Stephens (@0xsatoshi99)

See Wikipedia article (https://en.wikipedia.org/wiki/Genetic_algorithm)

References:
    - Holland, J.H. (1975). "Adaptation in Natural and Artificial Systems"
    - Goldberg, D.E. (1989). "Genetic Algorithms in Search, Optimization, and Machine Learning"
    - Mitchell, M. (1998). "An Introduction to Genetic Algorithms"

This implementation uses a genetic algorithm to evolve optimal paths from
start to goal while avoiding obstacles. The algorithm uses selection,
crossover, and mutation operators to evolve a population of candidate paths.
"""

import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import matplotlib.patches as patches
import signal
import sys

# Animation flag for consistency with other planners
show_animation = True


def signal_handler(sig, frame):
    """Handle Ctrl+C gracefully"""
    print("\nExiting...")
    plt.close("all")
    sys.exit(0)


signal.signal(signal.SIGINT, signal_handler)


class Individual:
    """Represents a single path (chromosome) in the population.
    
    Each individual encodes a path as a sequence of waypoints between
    start and goal. The fitness is based on path length and collision penalty.
    
    Attributes:
        genes: Array of waypoint positions shape (n_waypoints, 2)
        fitness: Fitness value (lower is better)
        path_length: Total path length
        collision_penalty: Penalty for obstacle collisions
    """
    
    def __init__(self, start, goal, n_waypoints, search_bounds):
        """Initialize individual with random waypoints.
        
        Args:
            start: Starting position [x, y]
            goal: Goal position [x, y]
            n_waypoints: Number of intermediate waypoints
            search_bounds: [(x_min, x_max), (y_min, y_max)]
        """
        self.start = np.array(start)
        self.goal = np.array(goal)
        self.n_waypoints = n_waypoints
        self.search_bounds = search_bounds
        
        # Generate random waypoints between start and goal
        self.genes = np.zeros((n_waypoints, 2))
        for i in range(n_waypoints):
            self.genes[i, 0] = np.random.uniform(search_bounds[0][0], search_bounds[0][1])
            self.genes[i, 1] = np.random.uniform(search_bounds[1][0], search_bounds[1][1])
        
        self.fitness = float('inf')
        self.path_length = 0.0
        self.collision_penalty = 0.0
    
    def get_full_path(self):
        """Get complete path including start and goal.
        
        Returns:
            Array of shape (n_waypoints + 2, 2) with full path
        """
        return np.vstack([self.start, self.genes, self.goal])
    
    def calculate_fitness(self, obstacles):
        """Calculate fitness based on path length and collisions.
        
        Fitness = path_length + collision_penalty
        Lower fitness is better.
        
        Args:
            obstacles: List of (x, y, radius) tuples
        """
        path = self.get_full_path()
        
        # Calculate total path length
        self.path_length = 0.0
        for i in range(len(path) - 1):
            self.path_length += np.linalg.norm(path[i+1] - path[i])
        
        # Calculate collision penalty
        self.collision_penalty = 0.0
        for i in range(len(path) - 1):
            start = path[i]
            end = path[i+1]
            for ox, oy, radius in obstacles:
                if self._check_collision(start, end, ox, oy, radius):
                    self.collision_penalty += 1000.0  # Heavy penalty
        
        self.fitness = self.path_length + self.collision_penalty
    
    def _check_collision(self, start, end, ox, oy, radius):
        """Check if line segment collides with circular obstacle.
        
        Uses point-to-line distance formula.
        
        Args:
            start: Start point of segment
            end: End point of segment
            ox, oy: Obstacle center
            radius: Obstacle radius
            
        Returns:
            True if collision detected
        """
        # Vector from start to end
        d = end - start
        # Vector from start to obstacle center
        f = start - np.array([ox, oy])
        
        a = np.dot(d, d)
        b = 2 * np.dot(f, d)
        c = np.dot(f, f) - radius * radius
        
        discriminant = b * b - 4 * a * c
        
        if discriminant < 0:
            return False
        
        discriminant = np.sqrt(discriminant)
        t1 = (-b - discriminant) / (2 * a)
        t2 = (-b + discriminant) / (2 * a)
        
        # Check if intersection occurs within segment
        if (0 <= t1 <= 1) or (0 <= t2 <= 1):
            return True
        
        # Check if obstacle center is close to endpoints
        if np.linalg.norm(start - np.array([ox, oy])) < radius:
            return True
        if np.linalg.norm(end - np.array([ox, oy])) < radius:
            return True
        
        return False


class GeneticAlgorithm:
    """Genetic Algorithm for path planning.
    
    Evolves a population of paths using selection, crossover, and mutation
    to find optimal collision-free paths.
    
    Attributes:
        start: Starting position
        goal: Goal position
        obstacles: List of obstacles
        population_size: Number of individuals in population
        n_generations: Maximum number of generations
        n_waypoints: Number of waypoints per path
        mutation_rate: Probability of mutation (0-1)
        crossover_rate: Probability of crossover (0-1)
        elite_size: Number of best individuals to preserve
        population: Current population
        best_individual: Best individual found
        generation: Current generation number
    """
    
    def __init__(self, start, goal, obstacles, search_bounds,
                 population_size=50, n_generations=100, n_waypoints=8,
                 mutation_rate=0.1, crossover_rate=0.8, elite_size=5):
        """Initialize genetic algorithm.
        
        Args:
            start: Starting position [x, y]
            goal: Goal position [x, y]
            obstacles: List of (x, y, radius) tuples
            search_bounds: [(x_min, x_max), (y_min, y_max)]
            population_size: Size of population
            n_generations: Maximum generations
            n_waypoints: Waypoints per individual
            mutation_rate: Mutation probability
            crossover_rate: Crossover probability
            elite_size: Number of elites to preserve
        """
        self.start = np.array(start)
        self.goal = np.array(goal)
        self.obstacles = obstacles
        self.search_bounds = search_bounds
        self.population_size = population_size
        self.n_generations = n_generations
        self.n_waypoints = n_waypoints
        self.mutation_rate = mutation_rate
        self.crossover_rate = crossover_rate
        self.elite_size = elite_size
        
        # Initialize population
        self.population = []
        for _ in range(population_size):
            individual = Individual(start, goal, n_waypoints, search_bounds)
            individual.calculate_fitness(obstacles)
            self.population.append(individual)
        
        self.population.sort(key=lambda x: x.fitness)
        self.best_individual = self.population[0]
        self.generation = 0
        self.fitness_history = []
    
    def selection(self):
        """Select parents using tournament selection.
        
        Returns:
            Two parent individuals
        """
        tournament_size = 5
        
        def tournament():
            competitors = np.random.choice(self.population, tournament_size, replace=False)
            return min(competitors, key=lambda x: x.fitness)
        
        parent1 = tournament()
        parent2 = tournament()
        return parent1, parent2
    
    def crossover(self, parent1, parent2):
        """Perform uniform crossover between two parents.
        
        Args:
            parent1: First parent
            parent2: Second parent
            
        Returns:
            Two offspring individuals
        """
        if np.random.rand() > self.crossover_rate:
            return parent1, parent2
        
        # Create offspring
        offspring1 = Individual(self.start, self.goal, self.n_waypoints, self.search_bounds)
        offspring2 = Individual(self.start, self.goal, self.n_waypoints, self.search_bounds)
        
        # Uniform crossover: randomly select genes from each parent
        for i in range(self.n_waypoints):
            if np.random.rand() < 0.5:
                offspring1.genes[i] = parent1.genes[i].copy()
                offspring2.genes[i] = parent2.genes[i].copy()
            else:
                offspring1.genes[i] = parent2.genes[i].copy()
                offspring2.genes[i] = parent1.genes[i].copy()
        
        return offspring1, offspring2
    
    def mutate(self, individual):
        """Apply Gaussian mutation to individual.
        
        Args:
            individual: Individual to mutate
        """
        for i in range(self.n_waypoints):
            if np.random.rand() < self.mutation_rate:
                # Gaussian mutation
                mutation = np.random.randn(2) * 2.0
                individual.genes[i] += mutation
                
                # Keep within bounds
                individual.genes[i, 0] = np.clip(
                    individual.genes[i, 0],
                    self.search_bounds[0][0],
                    self.search_bounds[0][1]
                )
                individual.genes[i, 1] = np.clip(
                    individual.genes[i, 1],
                    self.search_bounds[1][0],
                    self.search_bounds[1][1]
                )
    
    def evolve(self):
        """Perform one generation of evolution.
        
        Returns:
            True if should continue, False if completed
        """
        if self.generation >= self.n_generations:
            return False
        
        # Create new population
        new_population = []
        
        # Elitism: keep best individuals
        new_population.extend(self.population[:self.elite_size])
        
        # Generate offspring
        while len(new_population) < self.population_size:
            # Selection
            parent1, parent2 = self.selection()
            
            # Crossover
            offspring1, offspring2 = self.crossover(parent1, parent2)
            
            # Mutation
            self.mutate(offspring1)
            self.mutate(offspring2)
            
            # Evaluate fitness
            offspring1.calculate_fitness(self.obstacles)
            offspring2.calculate_fitness(self.obstacles)
            
            new_population.append(offspring1)
            if len(new_population) < self.population_size:
                new_population.append(offspring2)
        
        # Update population
        self.population = new_population
        self.population.sort(key=lambda x: x.fitness)
        
        # Update best individual
        if self.population[0].fitness < self.best_individual.fitness:
            self.best_individual = self.population[0]
        
        self.fitness_history.append(self.best_individual.fitness)
        self.generation += 1
        
        return True


def main():
    """Run Genetic Algorithm path planning demonstration.
    
    This function demonstrates GA-based path planning with:
    - Population of 50 individuals
    - 8 waypoints per path
    - Start at (-40, -40), Goal at (40, 40)
    - Multiple circular obstacles
    - 100 generations of evolution
    """
    print("Genetic Algorithm Path Planning")
    print("=" * 50)
    
    # Configuration
    START = np.array([-40.0, -40.0])
    GOAL = np.array([40.0, 40.0])
    SEARCH_BOUNDS = [(-50, 50), (-50, 50)]
    
    # Obstacles: (x, y, radius)
    OBSTACLES = [
        (0, 0, 8),
        (-20, 20, 6),
        (20, -20, 6),
        (20, 20, 7),
        (-20, -20, 7),
        (0, 30, 5),
        (0, -30, 5),
        (30, 0, 5),
        (-30, 0, 5),
    ]
    
    # GA parameters
    POPULATION_SIZE = 50
    N_GENERATIONS = 100
    N_WAYPOINTS = 8
    MUTATION_RATE = 0.1
    CROSSOVER_RATE = 0.8
    ELITE_SIZE = 5
    
    # Initialize GA
    ga = GeneticAlgorithm(
        START, GOAL, OBSTACLES, SEARCH_BOUNDS,
        population_size=POPULATION_SIZE,
        n_generations=N_GENERATIONS,
        n_waypoints=N_WAYPOINTS,
        mutation_rate=MUTATION_RATE,
        crossover_rate=CROSSOVER_RATE,
        elite_size=ELITE_SIZE
    )
    
    print(f"Population Size: {POPULATION_SIZE}")
    print(f"Generations: {N_GENERATIONS}")
    print(f"Waypoints: {N_WAYPOINTS}")
    print(f"Initial Best Fitness: {ga.best_individual.fitness:.2f}")
    print("=" * 50)
    
    if show_animation:
        # Setup plot
        fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(16, 7))
        
        # Left plot: Path visualization
        ax1.set_xlim(SEARCH_BOUNDS[0])
        ax1.set_ylim(SEARCH_BOUNDS[1])
        ax1.set_aspect('equal')
        ax1.grid(True, alpha=0.3)
        ax1.set_xlabel('X')
        ax1.set_ylabel('Y')
        ax1.set_title('Genetic Algorithm Path Planning')
        
        # Draw obstacles
        for ox, oy, r in OBSTACLES:
            circle = patches.Circle((ox, oy), r, color='gray', alpha=0.7)
            ax1.add_patch(circle)
        
        # Draw start and goal
        ax1.plot(START[0], START[1], 'go', markersize=15, label='Start')
        ax1.plot(GOAL[0], GOAL[1], 'r*', markersize=20, label='Goal')
        
        # Initialize plot elements
        population_lines = [ax1.plot([], [], 'b-', lw=0.5, alpha=0.2)[0] 
                          for _ in range(min(10, POPULATION_SIZE))]
        best_path_line = ax1.plot([], [], 'r-', lw=3, alpha=0.8, label='Best Path')[0]
        waypoints_scatter = ax1.scatter([], [], c='red', s=50, alpha=0.6, zorder=5)
        
        generation_text = ax1.text(
            0.02, 0.98, '', transform=ax1.transAxes,
            fontsize=12, verticalalignment='top',
            bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.8)
        )
        
        ax1.legend(loc='upper right')
        
        # Right plot: Fitness history
        ax2.set_xlabel('Generation')
        ax2.set_ylabel('Best Fitness')
        ax2.set_title('Evolution Progress')
        ax2.grid(True, alpha=0.3)
        fitness_line = ax2.plot([], [], 'b-', lw=2)[0]
        
        def animate(frame):
            """Animation function"""
            if not ga.evolve():
                return population_lines + [best_path_line, waypoints_scatter, 
                                          generation_text, fitness_line]
            
            # Update population paths (show top 10)
            for i, line in enumerate(population_lines):
                if i < len(ga.population):
                    path = ga.population[i].get_full_path()
                    line.set_data(path[:, 0], path[:, 1])
            
            # Update best path
            best_path = ga.best_individual.get_full_path()
            best_path_line.set_data(best_path[:, 0], best_path[:, 1])
            
            # Update waypoints
            waypoints = ga.best_individual.genes
            waypoints_scatter.set_offsets(waypoints)
            
            # Update text
            generation_text.set_text(
                f'Generation: {ga.generation}/{N_GENERATIONS}\n'
                f'Best Fitness: {ga.best_individual.fitness:.2f}\n'
                f'Path Length: {ga.best_individual.path_length:.2f}\n'
                f'Collisions: {int(ga.best_individual.collision_penalty/1000)}'
            )
            
            # Update fitness history
            if ga.fitness_history:
                fitness_line.set_data(range(len(ga.fitness_history)), ga.fitness_history)
                ax2.set_xlim(0, max(10, len(ga.fitness_history)))
                ax2.set_ylim(0, max(ga.fitness_history) * 1.1)
            
            return population_lines + [best_path_line, waypoints_scatter, 
                                      generation_text, fitness_line]
        
        # Create animation
        anim = animation.FuncAnimation(
            fig, animate, frames=N_GENERATIONS,
            interval=50, blit=True, repeat=False
        )
        
        plt.tight_layout()
        plt.show()
        
        return anim
    else:
        # Run without animation
        print("Running GA without animation...")
        while ga.evolve():
            if ga.generation % 10 == 0:
                print(f"Generation {ga.generation}: Best Fitness = {ga.best_individual.fitness:.2f}")
        
        print(f"\nGA completed after {ga.generation} generations")
        print(f"Final Best Fitness: {ga.best_individual.fitness:.2f}")
        print(f"Path Length: {ga.best_individual.path_length:.2f}")
        print(f"Collisions: {int(ga.best_individual.collision_penalty/1000)}")
        
        return None


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\nProgram interrupted by user")
        plt.close("all")
        sys.exit(0)
