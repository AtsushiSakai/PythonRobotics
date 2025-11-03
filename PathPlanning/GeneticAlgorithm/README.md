# Genetic Algorithm Path Planning

## Overview

This module implements path planning using a Genetic Algorithm (GA), an evolutionary optimization technique inspired by natural selection. The algorithm evolves a population of candidate paths to find optimal collision-free routes from start to goal.

## Algorithm Description

### Genetic Algorithm Basics

Genetic Algorithms are search heuristics that mimic the process of natural evolution:

1. **Population**: A set of candidate solutions (paths)
2. **Selection**: Choose parents based on fitness
3. **Crossover**: Combine parent genes to create offspring
4. **Mutation**: Introduce random variations
5. **Evolution**: Repeat for multiple generations

### Path Representation

Each individual (chromosome) represents a complete path:
- **Genes**: Sequence of waypoint coordinates
- **Path**: Start → Waypoint₁ → ... → Waypoint_n → Goal

### Fitness Function

```
Fitness = Path_Length + Collision_Penalty
```

- **Path Length**: Sum of Euclidean distances between consecutive points
- **Collision Penalty**: Heavy penalty (1000) for each obstacle collision
- **Goal**: Minimize fitness value

### Genetic Operators

#### 1. Selection (Tournament)
- Randomly select k individuals
- Choose the best among them
- Provides selection pressure while maintaining diversity

#### 2. Crossover (Uniform)
- Randomly exchange genes between parents
- Crossover rate: 0.8 (80% probability)
- Creates offspring with mixed characteristics

#### 3. Mutation (Gaussian)
- Add random Gaussian noise to waypoints
- Mutation rate: 0.1 (10% probability)
- Maintains population diversity

#### 4. Elitism
- Preserve top 5 individuals unchanged
- Prevents loss of best solutions

## Features

- **Collision Detection**: Line-circle intersection for obstacle avoidance
- **Adaptive Evolution**: Population evolves toward better solutions
- **Real-time Visualization**: Shows evolution progress
- **Fitness Tracking**: Plots fitness improvement over generations

## Parameters

| Parameter | Default | Description |
|-----------|---------|-------------|
| `population_size` | 50 | Number of individuals in population |
| `n_generations` | 100 | Maximum number of generations |
| `n_waypoints` | 8 | Number of intermediate waypoints |
| `mutation_rate` | 0.1 | Probability of mutation (0-1) |
| `crossover_rate` | 0.8 | Probability of crossover (0-1) |
| `elite_size` | 5 | Number of best individuals to preserve |

## Usage

```python
from genetic_algorithm import GeneticAlgorithm
import numpy as np

# Define problem
start = np.array([-40.0, -40.0])
goal = np.array([40.0, 40.0])
obstacles = [(0, 0, 8), (-20, 20, 6), (20, -20, 6)]  # (x, y, radius)
search_bounds = [(-50, 50), (-50, 50)]

# Create GA planner
ga = GeneticAlgorithm(
    start, goal, obstacles, search_bounds,
    population_size=50,
    n_generations=100,
    n_waypoints=8
)

# Evolve population
while ga.evolve():
    pass

# Get best path
best_path = ga.best_individual.get_full_path()
print(f"Best fitness: {ga.best_individual.fitness:.2f}")
```

## Visualization

The animation shows:
- **Left Panel**: Path evolution
  - Gray circles: Obstacles
  - Green circle: Start position
  - Red star: Goal position
  - Blue lines: Population paths (top 10)
  - Red line: Best path found
  - Red dots: Waypoints of best path

- **Right Panel**: Fitness history
  - Shows improvement over generations
  - Typically converges after 50-80 generations

## Algorithm Complexity

- **Time**: O(G × P × W × O)
  - G: Number of generations
  - P: Population size
  - W: Number of waypoints
  - O: Number of obstacles

- **Space**: O(P × W)

## Advantages

1. **Global Search**: Explores entire search space
2. **Parallelizable**: Fitness evaluations are independent
3. **Flexible**: Easy to modify fitness function
4. **Robust**: Handles complex obstacle configurations

## Limitations

1. **Computational Cost**: Requires many fitness evaluations
2. **Parameter Tuning**: Performance depends on parameter settings
3. **No Optimality Guarantee**: Heuristic method
4. **Premature Convergence**: May get stuck in local optima

## References

1. Holland, J.H. (1975). "Adaptation in Natural and Artificial Systems"
2. Goldberg, D.E. (1989). "Genetic Algorithms in Search, Optimization, and Machine Learning"
3. Mitchell, M. (1998). "An Introduction to Genetic Algorithms"
4. Eiben, A.E.; Smith, J.E. (2015). "Introduction to Evolutionary Computing"

## Example Output

```
Genetic Algorithm Path Planning
==================================================
Population Size: 50
Generations: 100
Waypoints: 8
Initial Best Fitness: 245.67
==================================================
Generation 10: Best Fitness = 156.23
Generation 20: Best Fitness = 134.89
Generation 30: Best Fitness = 128.45
Generation 40: Best Fitness = 125.12
...
GA completed after 100 generations
Final Best Fitness: 122.34
Path Length: 122.34
Collisions: 0
```

## See Also

- [Particle Swarm Optimization](../ParticleSwarmOptimization/)
- [A* Algorithm](../AStar/)
- [RRT*](../RRTStar/)
