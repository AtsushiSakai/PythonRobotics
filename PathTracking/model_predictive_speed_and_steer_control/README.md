# Model Predictive Control (MPC) Overview

Model Predictive Control (MPC) is an advanced control strategy that uses a dynamic model of the system to predict and optimize control actions. It solves an optimization problem at each control step to determine the optimal control inputs over a future time horizon. MPC is particularly useful for systems with constraints and is widely used in robotics, automotive, and process industries.

### Key Components of MPC

1. **Prediction Model:**
   - MPC uses a model of the system to predict future states over a specified horizon.
   - The prediction model in this case is based on the kinematic bicycle model, which describes the motion of a vehicle.

2. **Cost Function:**
   - The cost function quantifies the error between the predicted states and the desired trajectory.
   - The objective is to minimize this cost, which typically includes terms for tracking error, control effort, and smoothness.

3. **Constraints:**
   - Constraints on states (e.g., position, velocity) and control inputs (e.g., steering angle, acceleration) are included to ensure feasible and safe operation.

4. **Optimization Solver:**
   - An optimization solver is used to find the control inputs that minimize the cost function while satisfying the constraints.


---

## Overview
![MPC_3x3](https://github.com/AtsushiSakai/PythonRobotics/assets/113830751/c8ae9e27-0e4a-4a95-ae9e-92898744904d)


The models used in MPC are generally intended to represent the behavior of complex and simple dynamical systems. The additional complexity of the MPC control algorithm is not generally needed to provide adequate control of simple systems, which are often controlled well by generic PID controllers. Common dynamic characteristics that are difficult for PID controllers include large time delays and high-order dynamics.

MPC models predict the change in the dependent variables of the modeled system that will be caused by changes in the independent variables. In a chemical process, independent variables that can be adjusted by the controller are often either the setpoints of regulatory PID controllers (pressure, flow, temperature, etc.) or the final control element (valves, dampers, etc.). Independent variables that cannot be adjusted by the controller are used as disturbances. Dependent variables in these processes are other measurements that represent either control objectives or process constraints.

MPC uses the current plant measurements, the current dynamic state of the process, the MPC models, and the process variable targets and limits to calculate future changes in the dependent variables. These changes are calculated to hold the dependent variables close to target while honoring constraints on both independent and dependent variables. The MPC typically sends out only the first change in each independent variable to be implemented, and repeats the calculation when the next change is required.

While many real processes are not linear, they can often be considered to be approximately linear over a small operating range. Linear MPC approaches are used in the majority of applications with the feedback mechanism of the MPC compensating for prediction errors due to structural mismatch between the model and the process. In model predictive controllers that consist only of linear models, the superposition principle of linear algebra enables the effect of changes in multiple independent variables to be added together to predict the response of the dependent variables. This simplifies the control problem to a series of direct matrix algebra calculations that are fast and robust.

When linear models are not sufficiently accurate to represent the real process nonlinearities, several approaches can be used. In some cases, the process variables can be transformed before and/or after the linear MPC model to reduce the nonlinearity. The process can be controlled with nonlinear MPC that uses a nonlinear model directly in the control application. The nonlinear model may be in the form of an empirical data fit (e.g. artificial neural networks) or a high-fidelity dynamic model based on fundamental mass and energy balances. The nonlinear model may be linearized to derive a Kalman filter or specify a model for linear MPC.

An algorithmic study by El-Gherwi, Budman, and El Kamel shows that utilizing a dual-mode approach can provide significant reduction in online computations while maintaining comparative performance to a non-altered implementation. The proposed algorithm solves N convex optimization problems in parallel based on exchange of information among controllers.


**Principles of MPC**
Model predictive control is a multivariable control algorithm that uses:

an internal dynamic model of the process
a cost function J over the receding horizon
an optimization algorithm minimizing the cost function J using the control input u
An example of a quadratic cost function for optimization is given by:

![image](https://github.com/AtsushiSakai/PythonRobotics/assets/113830751/0023dd94-d4f3-400f-bd80-7a1e237295d9)

without violating constraints (low/high limits) with:

${\displaystyle x_{i}}: {\displaystyle}𝑖^{th}$: controlled variable\
${\displaystyle r_{i}}: {\displaystyle}𝑖^{th}$: reference variable\
${\displaystyle w_{x_{i}}}$: weighting coefficient reflecting the relative importance of $𝑥_𝑖{\displaystyle}$

etc.

---

### Code Explanation

The provided code implements MPC for speed and steering control of a vehicle. Here's a detailed explanation of the key sections:

1. **Imports and Definitions:**
   ```python
   import os
   import sys
   import math
   import numpy as np
   import matplotlib.pyplot as plt
   import cvxpy
   ```

2. **Vehicle Parameters:**
   ```python
   NX = 4  # state vector [x, y, v, yaw]
   NU = 2  # input vector [accel, steer]
   T = 5  # horizon length
   DT = 0.2  # time step
   ```

3. **State Update Function:**
   ```python
   def update_state(state, accel, delta):
       # Update vehicle state based on current state, acceleration, and steering angle
       # Kinematic bicycle model
       ...
       return state
   ```

4. **MPC Solver:**
   ```python
   def linear_mpc_control(xref, x0, dref):
       # Solve MPC optimization problem
       ...
       return oa, odelta, ox, oy, ov, oyaw
   ```

5. **Simulation Loop:**
   ```python
   while T >= 1.0:
       # Main simulation loop
       ...
       # Solve MPC problem
       ...
       # Update vehicle state
       ...
   ```

### Detailed Steps in the Code

1. **State Update Function:**
   The `update_state` function updates the vehicle's state using the kinematic bicycle model. It calculates the new position `(x, y)`, velocity `v`, and yaw angle `yaw` based on the current state, acceleration `accel`, and steering angle `delta`.

2. **MPC Optimization:**
   The `linear_mpc_control` function formulates and solves the MPC problem using the `cvxpy` optimization library. The optimization variables are acceleration and steering angle over the prediction horizon. The cost function includes terms for tracking error and control effort, which are minimized subject to vehicle dynamics and constraints.

3. **Simulation Loop:**
   The main simulation loop runs until a stopping condition is met. At each time step:
   - The reference trajectory is generated.
   - The MPC problem is solved to get the optimal control inputs.
   - The vehicle state is updated using the optimal control inputs.
   - The results are plotted for visualization.
