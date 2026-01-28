"""Base Bayes Filter class for localization algorithms.

This module provides an abstract base class for Bayesian filtering algorithms
used in robot localization. It defines the common interface and methods that
should be implemented by specific filter types such as Extended Kalman Filter,
Particle Filter, Unscented Kalman Filter, etc.

Issue #1277: Code improvement in Localization dir
Reference: https://github.com/AtsushiSakai/PythonRobotics/issues/1277
"""

from abc import ABC, abstractmethod
import numpy as np


class BayesFilter(ABC):
    """Abstract base class for Bayesian filters.
    
    This class defines the common interface for all Bayesian filtering algorithms
    used in localization. All specific filter implementations (EKF, UKF, PF, etc.)
    should inherit from this class and implement the required abstract methods.
    
    Attributes:
        state_dim (int): Dimension of the state vector
        observation_dim (int): Dimension of the observation vector
    """
    
    def __init__(self, state_dim, observation_dim):
        """Initialize the Bayes filter.
        
        Args:
            state_dim (int): Dimension of the state vector
            observation_dim (int): Dimension of the observation vector
        """
        self.state_dim = state_dim
        self.observation_dim = observation_dim
    
    @abstractmethod
    def predict(self, control_input, dt):
        """Prediction step of the Bayesian filter.
        
        Propagate the belief through the motion model.
        
        Args:
            control_input: Control input (e.g., velocity commands)
            dt (float): Time step
            
        Returns:
            Updated belief after prediction
        """
        pass
    
    @abstractmethod
    def update(self, observation):
        """Update step of the Bayesian filter.
        
        Update the belief based on the observation.
        
        Args:
            observation: Sensor observation
            
        Returns:
            Updated belief after measurement update
        """
        pass
    
    @abstractmethod
    def get_state(self):
        """Get the current state estimate.
        
        Returns:
            Current state estimate
        """
        pass
    
    @abstractmethod
    def get_covariance(self):
        """Get the current uncertainty/covariance.
        
        Returns:
            Current covariance matrix or uncertainty measure
        """
        pass
    
    def reset(self):
        """Reset the filter to initial state.
        
        This method can be overridden by subclasses if needed.
        """
        pass
