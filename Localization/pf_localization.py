"""Particle Filter Localization - Refactored to use BayesFilter base class.

This module provides a Particle Filter implementation that inherits from
the BayesFilter base class, creating a consistent API across all localization filters.

Issue #1277: Code improvement in Localization dir
Reference: https://github.com/AtsushiSakai/PythonRobotics/issues/1277
"""

import numpy as np
from bayes_filter import BayesFilter


class ParticleFilter(BayesFilter):
    """Particle Filter for robot localization.
    
    This class implements the Particle Filter algorithm (also known as Sequential
    Monte Carlo) as a subclass of the BayesFilter base class. It represents the
    belief as a set of weighted particles, making it suitable for non-Gaussian
    and multimodal distributions.
    
    Attributes:
        num_particles (int): Number of particles to use
        particles (np.ndarray): Array of particle states [num_particles x state_dim]
        weights (np.ndarray): Array of particle weights [num_particles]
    """
    
    def __init__(self, state_dim, observation_dim, num_particles=100):
        """Initialize the Particle Filter.
        
        Args:
            state_dim (int): Dimension of the state vector
            observation_dim (int): Dimension of the observation vector
            num_particles (int): Number of particles to use
        """
        super().__init__(state_dim, observation_dim)
        
        self.num_particles = num_particles
        
        # Initialize particles uniformly
        self.particles = np.zeros((num_particles, state_dim))
        
        # Initialize weights uniformly
        self.weights = np.ones(num_particles) / num_particles
        
        # Process and measurement noise parameters
        self.process_noise_std = 0.1
        self.measurement_noise_std = 0.1
    
    def initialize_particles(self, initial_state, initial_cov):
        """Initialize particles around an initial state with covariance.
        
        Args:
            initial_state (np.ndarray): Initial state estimate
            initial_cov (np.ndarray): Initial covariance matrix
        """
        for i in range(self.num_particles):
            self.particles[i] = np.random.multivariate_normal(initial_state, initial_cov)
        
        # Reset weights uniformly
        self.weights = np.ones(self.num_particles) / self.num_particles
    
    def set_noise_parameters(self, process_noise_std, measurement_noise_std):
        """Set the process and measurement noise standard deviations.
        
        Args:
            process_noise_std (float): Process noise standard deviation
            measurement_noise_std (float): Measurement noise standard deviation
        """
        self.process_noise_std = process_noise_std
        self.measurement_noise_std = measurement_noise_std
    
    def predict(self, control_input, dt):
        """Prediction step of the Particle Filter.
        
        Propagates each particle forward according to the motion model with noise.
        
        Args:
            control_input (np.ndarray): Control input (e.g., velocity commands)
            dt (float): Time step
            
        Returns:
            np.ndarray: Particles after prediction
        """
        # Propagate each particle
        for i in range(self.num_particles):
            # Apply motion model
            self.particles[i] = self.motion_model(self.particles[i], control_input, dt)
            
            # Add process noise
            noise = np.random.randn(self.state_dim) * self.process_noise_std
            self.particles[i] += noise
        
        return self.particles
    
    def update(self, observation, landmark_pos=None):
        """Update step of the Particle Filter.
        
        Updates particle weights based on observation likelihood and performs resampling.
        
        Args:
            observation (np.ndarray): Sensor observation
            landmark_pos (np.ndarray, optional): Known landmark positions
            
        Returns:
            np.ndarray: Particles after update and resampling
        """
        # Update weights based on observation likelihood
        for i in range(self.num_particles):
            # Predicted observation for this particle
            z_pred = self.observation_model(self.particles[i], landmark_pos)
            
            # Compute likelihood using Gaussian
            diff = observation - z_pred
            likelihood = self._gaussian_likelihood(diff, self.measurement_noise_std)
            
            # Update weight
            self.weights[i] *= likelihood
        
        # Normalize weights
        weight_sum = np.sum(self.weights)
        if weight_sum > 0:
            self.weights /= weight_sum
        else:
            # If all weights are zero, reset uniformly
            self.weights = np.ones(self.num_particles) / self.num_particles
        
        # Resample particles
        self._resample()
        
        return self.particles
    
    def _gaussian_likelihood(self, diff, std):
        """Compute Gaussian likelihood.
        
        Args:
            diff (np.ndarray): Difference between observation and prediction
            std (float): Standard deviation
            
        Returns:
            float: Likelihood value
        """
        # Multivariate Gaussian likelihood
        exponent = -0.5 * np.dot(diff, diff) / (std ** 2)
        return np.exp(exponent)
    
    def _resample(self):
        """Resample particles based on weights (low variance resampling)."""
        # Low variance resampling
        new_particles = np.zeros_like(self.particles)
        
        # Cumulative sum of weights
        cumsum = np.cumsum(self.weights)
        
        # Random starting point
        r = np.random.uniform(0, 1.0 / self.num_particles)
        
        i = 0
        for j in range(self.num_particles):
            u = r + j / self.num_particles
            
            # Find corresponding particle
            while u > cumsum[i]:
                i += 1
                if i >= self.num_particles:
                    i = self.num_particles - 1
                    break
            
            new_particles[j] = self.particles[i]
        
        self.particles = new_particles
        
        # Reset weights uniformly after resampling
        self.weights = np.ones(self.num_particles) / self.num_particles
    
    def get_state(self):
        """Get the current state estimate (weighted mean of particles).
        
        Returns:
            np.ndarray: Current state estimate
        """
        # Weighted mean of particles
        return np.average(self.particles, weights=self.weights, axis=0)
    
    def get_covariance(self):
        """Get the current covariance (weighted covariance of particles).
        
        Returns:
            np.ndarray: Current covariance matrix
        """
        # Weighted mean
        mean = self.get_state()
        
        # Weighted covariance
        cov = np.zeros((self.state_dim, self.state_dim))
        for i in range(self.num_particles):
            diff = self.particles[i] - mean
            cov += self.weights[i] * np.outer(diff, diff)
        
        return cov
    
    def reset(self):
        """Reset the filter to initial state."""
        # Reset particles to origin
        self.particles = np.zeros((self.num_particles, self.state_dim))
        
        # Reset weights uniformly
        self.weights = np.ones(self.num_particles) / self.num_particles
    
    def get_effective_particles(self):
        """Get the effective number of particles (useful for adaptive resampling).
        
        Returns:
            float: Effective number of particles
        """
        return 1.0 / np.sum(self.weights ** 2)
    
    # Motion and observation models - to be customized for specific applications
    
    def motion_model(self, x, u, dt):
        """Motion model function.
        
        Default implementation: simple kinematic model.
        Override this for specific robot models.
        
        Args:
            x (np.ndarray): Current state
            u (np.ndarray): Control input
            dt (float): Time step
            
        Returns:
            np.ndarray: Predicted next state
        """
        # Simple kinematic model for 2D robot [x, y, theta]
        # u = [v, omega] (linear velocity, angular velocity)
        if len(x) >= 3 and len(u) >= 2:
            x_next = x.copy()
            x_next[0] = x[0] + u[0] * np.cos(x[2]) * dt  # x position
            x_next[1] = x[1] + u[0] * np.sin(x[2]) * dt  # y position
            x_next[2] = x[2] + u[1] * dt  # orientation
            return x_next
        return x
    
    def observation_model(self, x, landmark_pos=None):
        """Observation model function.
        
        Default implementation: return state directly.
        Override this for specific sensor models.
        
        Args:
            x (np.ndarray): Current state
            landmark_pos (np.ndarray, optional): Known landmark positions
            
        Returns:
            np.ndarray: Predicted observation
        """
        # Simple identity observation model
        return x[:self.observation_dim]
