"""Extended Kalman Filter Localization - Refactored to use BayesFilter base class.

This module provides an Extended Kalman Filter implementation that inherits from
the BayesFilter base class, creating a consistent API across all localization filters.

Issue #1277: Code improvement in Localization dir
Reference: https://github.com/AtsushiSakai/PythonRobotics/issues/1277
"""

import numpy as np
from bayes_filter import BayesFilter


class ExtendedKalmanFilter(BayesFilter):
    """Extended Kalman Filter for robot localization.
    
    This class implements the Extended Kalman Filter algorithm as a subclass
    of the BayesFilter base class. It maintains a Gaussian belief represented
    by a mean (state) and covariance matrix.
    
    Attributes:
        x (np.ndarray): State estimate (mean of Gaussian belief)
        P (np.ndarray): Covariance matrix (uncertainty)
        Q (np.ndarray): Process noise covariance
        R (np.ndarray): Measurement noise covariance
    """
    
    def __init__(self, state_dim, observation_dim, initial_state=None, initial_cov=None):
        """Initialize the Extended Kalman Filter.
        
        Args:
            state_dim (int): Dimension of the state vector
            observation_dim (int): Dimension of the observation vector
            initial_state (np.ndarray, optional): Initial state estimate
            initial_cov (np.ndarray, optional): Initial covariance matrix
        """
        super().__init__(state_dim, observation_dim)
        
        # Initialize state and covariance
        if initial_state is not None:
            self.x = initial_state.copy()
        else:
            self.x = np.zeros(state_dim)
            
        if initial_cov is not None:
            self.P = initial_cov.copy()
        else:
            self.P = np.eye(state_dim)
        
        # Initialize process and measurement noise covariances
        self.Q = np.eye(state_dim) * 0.1  # Default process noise
        self.R = np.eye(observation_dim) * 0.1  # Default measurement noise
    
    def set_noise_covariances(self, Q, R):
        """Set the process and measurement noise covariances.
        
        Args:
            Q (np.ndarray): Process noise covariance
            R (np.ndarray): Measurement noise covariance
        """
        self.Q = Q.copy()
        self.R = R.copy()
    
    def predict(self, control_input, dt):
        """Prediction step of the EKF.
        
        Propagates the state estimate and covariance forward using the motion model.
        
        Args:
            control_input (np.ndarray): Control input (e.g., velocity commands)
            dt (float): Time step
            
        Returns:
            tuple: (predicted_state, predicted_covariance)
        """
        # Motion model: x_t = f(x_{t-1}, u_t)
        # This is a simple motion model - can be overridden for specific robots
        F = self.jacobian_motion_model(self.x, control_input, dt)
        
        # Predict state
        self.x = self.motion_model(self.x, control_input, dt)
        
        # Predict covariance
        self.P = F @ self.P @ F.T + self.Q
        
        return self.x, self.P
    
    def update(self, observation, landmark_pos=None):
        """Update step of the EKF.
        
        Updates the state estimate and covariance using the observation.
        
        Args:
            observation (np.ndarray): Sensor observation
            landmark_pos (np.ndarray, optional): Known landmark positions for observation model
            
        Returns:
            tuple: (updated_state, updated_covariance)
        """
        # Observation model: z_t = h(x_t)
        H = self.jacobian_observation_model(self.x, landmark_pos)
        
        # Predicted observation
        z_pred = self.observation_model(self.x, landmark_pos)
        
        # Innovation (measurement residual)
        y = observation - z_pred
        
        # Innovation covariance
        S = H @ self.P @ H.T + self.R
        
        # Kalman gain
        K = self.P @ H.T @ np.linalg.inv(S)
        
        # Update state
        self.x = self.x + K @ y
        
        # Update covariance
        I = np.eye(self.state_dim)
        self.P = (I - K @ H) @ self.P
        
        return self.x, self.P
    
    def get_state(self):
        """Get the current state estimate.
        
        Returns:
            np.ndarray: Current state estimate
        """
        return self.x.copy()
    
    def get_covariance(self):
        """Get the current covariance matrix.
        
        Returns:
            np.ndarray: Current covariance matrix
        """
        return self.P.copy()
    
    def reset(self):
        """Reset the filter to initial state."""
        self.x = np.zeros(self.state_dim)
        self.P = np.eye(self.state_dim)
    
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
    
    def jacobian_motion_model(self, x, u, dt):
        """Jacobian of the motion model.
        
        Args:
            x (np.ndarray): Current state
            u (np.ndarray): Control input
            dt (float): Time step
            
        Returns:
            np.ndarray: Jacobian matrix F
        """
        # Jacobian for simple kinematic model
        F = np.eye(self.state_dim)
        if len(x) >= 3 and len(u) >= 2:
            F[0, 2] = -u[0] * np.sin(x[2]) * dt
            F[1, 2] = u[0] * np.cos(x[2]) * dt
        return F
    
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
    
    def jacobian_observation_model(self, x, landmark_pos=None):
        """Jacobian of the observation model.
        
        Args:
            x (np.ndarray): Current state
            landmark_pos (np.ndarray, optional): Known landmark positions
            
        Returns:
            np.ndarray: Jacobian matrix H
        """
        # Jacobian for identity observation model
        H = np.zeros((self.observation_dim, self.state_dim))
        H[:self.observation_dim, :self.observation_dim] = np.eye(self.observation_dim)
        return H
