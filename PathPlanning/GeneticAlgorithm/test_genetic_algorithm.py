"""
Test script for Genetic Algorithm path planning

author: Dennis Stephens (@0xsatoshi99)
"""

import numpy as np
import pytest
from genetic_algorithm import Individual, GeneticAlgorithm


def test_individual_initialization():
    """Test Individual initialization"""
    start = np.array([0.0, 0.0])
    goal = np.array([10.0, 10.0])
    n_waypoints = 5
    search_bounds = [(-10, 10), (-10, 10)]
    
    individual = Individual(start, goal, n_waypoints, search_bounds)
    
    assert individual.genes.shape == (n_waypoints, 2)
    assert np.array_equal(individual.start, start)
    assert np.array_equal(individual.goal, goal)
    assert individual.fitness == float('inf')
    assert individual.path_length == 0.0
    assert individual.collision_penalty == 0.0


def test_individual_full_path():
    """Test getting full path including start and goal"""
    start = np.array([0.0, 0.0])
    goal = np.array([10.0, 10.0])
    n_waypoints = 3
    search_bounds = [(-10, 10), (-10, 10)]
    
    individual = Individual(start, goal, n_waypoints, search_bounds)
    full_path = individual.get_full_path()
    
    assert full_path.shape == (n_waypoints + 2, 2)
    assert np.array_equal(full_path[0], start)
    assert np.array_equal(full_path[-1], goal)


def test_fitness_calculation_no_obstacles():
    """Test fitness calculation without obstacles"""
    start = np.array([0.0, 0.0])
    goal = np.array([10.0, 0.0])
    n_waypoints = 2
    search_bounds = [(-10, 10), (-10, 10)]
    
    individual = Individual(start, goal, n_waypoints, search_bounds)
    # Set waypoints in a straight line
    individual.genes[0] = np.array([3.0, 0.0])
    individual.genes[1] = np.array([7.0, 0.0])
    
    obstacles = []
    individual.calculate_fitness(obstacles)
    
    # Fitness should be approximately 10 (straight line distance)
    assert individual.collision_penalty == 0.0
    assert 9.0 < individual.fitness < 11.0


def test_fitness_calculation_with_obstacles():
    """Test fitness calculation with obstacles"""
    start = np.array([0.0, 0.0])
    goal = np.array([10.0, 0.0])
    n_waypoints = 1
    search_bounds = [(-10, 10), (-10, 10)]
    
    individual = Individual(start, goal, n_waypoints, search_bounds)
    # Place waypoint directly on obstacle path
    individual.genes[0] = np.array([5.0, 0.0])
    
    # Obstacle at (5, 0) with radius 2
    obstacles = [(5.0, 0.0, 2.0)]
    individual.calculate_fitness(obstacles)
    
    # Should have collision penalty
    assert individual.collision_penalty > 0


def test_collision_detection():
    """Test collision detection method"""
    start = np.array([0.0, 0.0])
    goal = np.array([10.0, 10.0])
    n_waypoints = 2
    search_bounds = [(-10, 10), (-10, 10)]
    
    individual = Individual(start, goal, n_waypoints, search_bounds)
    
    # Test collision with obstacle in path
    assert individual._check_collision(
        np.array([0.0, 0.0]),
        np.array([10.0, 0.0]),
        5.0, 0.0, 1.0
    ) is True
    
    # Test no collision
    assert individual._check_collision(
        np.array([0.0, 0.0]),
        np.array([10.0, 0.0]),
        5.0, 5.0, 1.0
    ) is False


def test_ga_initialization():
    """Test GeneticAlgorithm initialization"""
    start = np.array([0.0, 0.0])
    goal = np.array([10.0, 10.0])
    obstacles = [(5.0, 5.0, 2.0)]
    search_bounds = [(-10, 10), (-10, 10)]
    
    ga = GeneticAlgorithm(
        start, goal, obstacles, search_bounds,
        population_size=10,
        n_generations=5,
        n_waypoints=3
    )
    
    assert len(ga.population) == 10
    assert ga.generation == 0
    assert ga.best_individual is not None
    assert ga.best_individual.fitness < float('inf')


def test_selection():
    """Test tournament selection"""
    start = np.array([0.0, 0.0])
    goal = np.array([10.0, 10.0])
    obstacles = []
    search_bounds = [(-10, 10), (-10, 10)]
    
    ga = GeneticAlgorithm(
        start, goal, obstacles, search_bounds,
        population_size=20,
        n_waypoints=3
    )
    
    parent1, parent2 = ga.selection()
    
    assert parent1 in ga.population
    assert parent2 in ga.population


def test_crossover():
    """Test crossover operation"""
    start = np.array([0.0, 0.0])
    goal = np.array([10.0, 10.0])
    obstacles = []
    search_bounds = [(-10, 10), (-10, 10)]
    
    ga = GeneticAlgorithm(
        start, goal, obstacles, search_bounds,
        population_size=10,
        n_waypoints=4,
        crossover_rate=1.0  # Always crossover
    )
    
    parent1 = ga.population[0]
    parent2 = ga.population[1]
    
    offspring1, offspring2 = ga.crossover(parent1, parent2)
    
    assert offspring1.genes.shape == parent1.genes.shape
    assert offspring2.genes.shape == parent2.genes.shape


def test_mutation():
    """Test mutation operation"""
    start = np.array([0.0, 0.0])
    goal = np.array([10.0, 10.0])
    obstacles = []
    search_bounds = [(-10, 10), (-10, 10)]
    
    ga = GeneticAlgorithm(
        start, goal, obstacles, search_bounds,
        population_size=10,
        n_waypoints=4,
        mutation_rate=1.0  # Always mutate
    )
    
    individual = ga.population[0]
    original_genes = individual.genes.copy()
    
    ga.mutate(individual)
    
    # Genes should be different after mutation
    assert not np.array_equal(individual.genes, original_genes)
    
    # Genes should still be within bounds
    for i in range(len(individual.genes)):
        assert search_bounds[0][0] <= individual.genes[i, 0] <= search_bounds[0][1]
        assert search_bounds[1][0] <= individual.genes[i, 1] <= search_bounds[1][1]


def test_evolution():
    """Test evolution process"""
    start = np.array([0.0, 0.0])
    goal = np.array([10.0, 10.0])
    obstacles = [(5.0, 5.0, 1.0)]
    search_bounds = [(-10, 10), (-10, 10)]
    
    ga = GeneticAlgorithm(
        start, goal, obstacles, search_bounds,
        population_size=20,
        n_generations=10,
        n_waypoints=3
    )
    
    initial_fitness = ga.best_individual.fitness
    
    # Run a few generations
    for _ in range(5):
        ga.evolve()
    
    # Fitness should improve or stay the same
    assert ga.best_individual.fitness <= initial_fitness
    assert ga.generation == 5


def test_evolution_convergence():
    """Test that GA converges to a solution"""
    start = np.array([0.0, 0.0])
    goal = np.array([10.0, 0.0])
    obstacles = []  # No obstacles for simple test
    search_bounds = [(-10, 10), (-10, 10)]
    
    ga = GeneticAlgorithm(
        start, goal, obstacles, search_bounds,
        population_size=30,
        n_generations=50,
        n_waypoints=3
    )
    
    # Run evolution
    while ga.evolve():
        pass
    
    # Should find a reasonable solution
    assert ga.best_individual.fitness < 20.0  # Should be close to 10
    assert ga.best_individual.collision_penalty == 0.0


def test_elitism():
    """Test that elitism preserves best individuals"""
    start = np.array([0.0, 0.0])
    goal = np.array([10.0, 10.0])
    obstacles = []
    search_bounds = [(-10, 10), (-10, 10)]
    
    ga = GeneticAlgorithm(
        start, goal, obstacles, search_bounds,
        population_size=20,
        n_generations=10,
        n_waypoints=3,
        elite_size=3
    )
    
    # Get best individuals before evolution
    best_before = [ind.fitness for ind in ga.population[:3]]
    
    ga.evolve()
    
    # At least some of the best should be preserved
    best_after = [ind.fitness for ind in ga.population[:3]]
    
    # Best fitness should not get worse
    assert min(best_after) <= min(best_before)


if __name__ == "__main__":
    pytest.main([__file__, "-v"])
