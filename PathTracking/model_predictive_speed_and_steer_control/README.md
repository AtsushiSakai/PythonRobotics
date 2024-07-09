# Model Predictive Speed and Steer Control.

 
## Overview: What is Model Predictive Control?

> Model predictive control (MPC) is an optimal control technique in which the calculated control actions minimize a cost function for a constrained dynamical system over a finite, receding, horizon.

> At each time step, an MPC controller receives or estimates the current state of the plant. It then calculates the sequence of control actions that minimizes the cost over the horizon by solving a constrained optimization problem that relies on an internal plant model and depends on the current system state. The controller then applies to the plant only the first computed control action, disregarding the following ones. In the following time step the process repeats.

> The following picture shows the basic MPC control loop, with the controller receiving the measured outputs and disturbances from the plant, and using an internal prediction plant model to both estimate the state and to calculate a sequence of control moves that minimize a cost function over a given horizon. In general, minimizing the cost function involves reducing the error between the future plant outputs and a reference trajectory. The unmeasured disturbances only affect the plant.

**MPC basic control loop**

![Feedback loop with an MPC controller (consisting of an observer and an optimizer, both using a prediction model) connected to a plant. The controller receives a reference signal, the plant output and the measured disturbances, and outputs the plant control moves. Only the plant is affected by the unmeasured disturbances.](https://www.mathworks.com/help/mpc/gs/mpc-intro-structure.png)

> The lower part of the following picture shows in more detail the reference trajectory and the predicted plant outputs. The MPC controller uses its internal prediction model to predict the plant outputs over the prediction horizon _p_. The upper part of the picture shows the control moves planned by the MPC control as well as the first control move, which is the one actually applied to the plant. The control horizon is the number of planned moves, and might be smaller than the prediction horizon.


**Analogy overview**

Imagine walking in a dark room. You try to sense the surroundings, predict the best path in the direction of a goal, but take only one step at a time and repeat the cycle.

Similarly, the MPC process is like walking into a dark room. The essence of MPC is to optimize the manipulatable inputs and the forecasts of process behavior.

MPC is an iterative process of optimizing the predictions of robot states in the future limited horizon while manipulating inputs for a given horizon. The forecasting is achieved using the process model. Thus, a dynamic model is essential while implementing MPC. These process models are generally nonlinear, but for short periods of time, there are methods such as tailor expansion to linearize these models. 

These approximations in linearization or unpredictable changes in dynamic models might cause errors in forecasting. Thus, MPC only takes action on first computed control input and then recalculates the optimized forecasts based on feedback. This implies MPC is an iterative, model-based, predictive, optimal, and feedback based control strategy.


**_This project implements a Model Predictive Control (MPC) framework for speed and steering control of a vehicle using a kinematic bicycle model. MPC is an advanced control strategy that uses a dynamic model of the system to predict and optimize control actions. This README file explains the key concepts, principles, and provides a detailed code walkthrough._**
![MPC_3x3](https://github.com/AtsushiSakai/PythonRobotics/assets/113830751/c8ae9e27-0e4a-4a95-ae9e-92898744904d)

---

## Key Concepts

### Model Predictive Control (MPC)

MPC is a control strategy that solves an optimization problem at each control step to determine the optimal control inputs over a future time horizon. It involves:

- **Prediction Model:** Uses the kinematic bicycle model to predict future states.
- **Cost Function:** Minimizes the error between the predicted states and the desired trajectory.
- **Constraints:** Ensures feasible and safe operation by imposing constraints on states and control inputs.

### Kinematic Bicycle Model

The kinematic bicycle model describes the motion of a vehicle based on its velocity, steering angle, and physical parameters. It's used for state prediction in the MPC framework.

### How Does MPC Work?

MPC has three basic requirements to work. The first one is a cost function J, which describes the expected behavior of the robot. 

This generally involves parameters of comparison between different possibilities of actions, such as minimization of error from the reference trajectory, minimization of jerk, obstacle avoidance, etc.

![image](https://github.com/Esmail-ibraheem/Robotics/assets/113830751/6abf13f5-ca7a-49bc-9767-f370c9146cec)



Where

J: cost function

$x_t$: robot states at time t

$rt$: robot reference states at time t

$ut$: robot predicted input at time t

$Wt$ and $Wb$: Weights according to the requirement

The above cost function minimizes error from a reference trajectory as well as jerk caused by drastic deviations in inputs to the robot. 

The second requirement is a dynamic model of the robot. This dynamic model enables MPC to simulate states of a robot in a given horizon with different possibilities of inputs. The third is the optimization algorithm used to solve given optimization function J. Along with these requirements, MPC provides flexibility to mention certain constraints to be taken into consideration while performing optimization. These constraints can be the minimum and maximum value of states and inputs to the robot.

![image](https://github.com/Esmail-ibraheem/Robotics/assets/113830751/3adcf974-9b61-4245-8c6d-05a9efc55c16)


> [!NOTE]
> In order to understand the working of MPC consider the robot is at current time k in the simulated robot movement and has a reference trajectory that needs to be followed for a given horizon p. MPC takes current states of the robot as input and simulates possibilities of control inputs for time k to k+p. From different possibilities, MPC selects the best series of inputs that minimize the cost function. From this series of predicted control input, MPC then implements only the first input and repeats the cycle at time k+1. Due to these iterative cycles over the horizon taking one step at a time, MPC is also called receding horizon control. This receding control can be better observed in the given simulation where black markers represent desired trajectories and red markers represent forecasted trajectories from MPC.

---

## Code Explanation

### Imports and Definitions

```python
import os
import sys
import math
import numpy as np
import matplotlib.pyplot as plt
import cvxpy

NX = 4  # state vector [x, y, v, yaw]
NU = 2  # input vector [accel, steer]
T = 5  # horizon length
DT = 0.2  # time step
```

### Vehicle Parameters

Defines the state and input vectors, and the horizon length for the MPC.

### State Update Function

```python
def update_state(state, accel, delta):
    # Update vehicle state based on current state, acceleration, and steering angle
    # Kinematic bicycle model
    x, y, yaw, v = state
    x += v * math.cos(yaw) * DT
    y += v * math.sin(yaw) * DT
    yaw += v / WB * math.tan(delta) * DT
    v += accel * DT
    return [x, y, yaw, v]
```

Updates the vehicle's state using the kinematic bicycle model.

### MPC Solver

```python
def linear_mpc_control(xref, x0, dref):
    # Solve MPC optimization problem
    # Define optimization variables
    x = cvxpy.Variable((NX, T + 1))
    u = cvxpy.Variable((NU, T))
    
    # Define cost function
    cost = 0
    constraints = []
    for t in range(T):
        cost += cvxpy.quad_form(x[:, t] - xref[:, t], Q)
        cost += cvxpy.quad_form(u[:, t], R)
        if t < (T - 1):
            cost += cvxpy.quad_form(u[:, t + 1] - u[:, t], Rd)
        constraints += [x[:, t + 1] == A @ x[:, t] + B @ u[:, t]]
        constraints += [cvxpy.abs(u[1, t]) <= MAX_STEER]
    
    cost += cvxpy.quad_form(x[:, T] - xref[:, T], Qf)
    constraints += [x[:, 0] == x0]
    
    # Solve the problem
    prob = cvxpy.Problem(cvxpy.Minimize(cost), constraints)
    prob.solve()
    
    if prob.status == cvxpy.OPTIMAL or prob.status == cvxpy.OPTIMAL_INACCURATE:
        ox = np.array(x.value[0, :]).flatten()
        oy = np.array(x.value[1, :]).flatten()
        ov = np.array(x.value[2, :]).flatten()
        oyaw = np.array(x.value[3, :]).flatten()
        oa = np.array(u.value[0, :]).flatten()
        odelta = np.array(u.value[1, :]).flatten()
    else:
        oa, odelta, ox, oy, ov, oyaw = None, None, None, None, None, None
    
    return oa, odelta, ox, oy, ov, oyaw
```

Formulates and solves the MPC problem using the `cvxpy` optimization library.

### Simulation Loop

```python
while T >= 1.0:
    # Main simulation loop
    # Generate reference trajectory
    ...
    
    # Solve MPC problem
    oa, odelta, ox, oy, ov, oyaw = linear_mpc_control(xref, state, dref)
    
    # Update vehicle state
    state = update_state(state, oa[0], odelta[0])
    
    # Visualization and result plotting
    ...
```

Runs the main simulation loop, solving the MPC problem and updating the vehicle state at each time step.

---
## Detailed Explanation of Key Concepts

### Prediction Model

The prediction model uses the kinematic bicycle model to predict the future states of the vehicle. This model captures the vehicle dynamics based on its velocity, steering angle, and other physical parameters.

### Cost Function and Constraints

#### Cost Function

In the context of MPC, the cost function is a mathematical formulation used to evaluate the performance of a set of control actions. The goal is to minimize this cost function over a given prediction horizon. Here's a deeper look into the components of the cost function used in this implementation:

1. **State Error Cost**: This term penalizes deviations of the predicted states from the reference states (desired trajectory).
   $$\text{cost} += \sum_{t=0}^{T} \left( x[:, t] - x_{ref}[:, t] \right)^T Q \left( x[:, t] - x_{ref}[:, t] \right)$$
   where $( x )$ is the predicted state vector, $( x_{ref} )$ is the reference state vector, and $( Q )$ is a weighting matrix that determines the importance of state deviations.

2. **Control Effort Cost**: This term penalizes the magnitude of control inputs to avoid excessive control actions.
   $$\text{cost} += \sum_{t=0}^{T-1} u[:, t]^T R u[:, t]$$
   where $( u )$ is the control input vector, and $( R )$ is a weighting matrix for control efforts.

3. **Control Change Cost**: This term penalizes large changes in control inputs between consecutive time steps to ensure smooth control actions.
   $$\text{cost} += \sum_{t=0}^{T-2} (u[:, t+1] - u[:, t])^T R_d (u[:, t+1] - u[:, t])$$
   where $( R_d )$ is a weighting matrix for control input changes.

4. **Terminal State Cost**: This term penalizes the deviation of the final predicted state from the reference state at the end of the prediction horizon.
  $$\text{cost} += (x[:, T] - x_{ref}[:, T])^T Q_f (x[:, T] - x_{ref}[:, T])$$
   where $( Q_f )$ is the terminal state weighting matrix.

#### Constraints

Constraints in MPC are used to ensure that the control actions and resulting states are feasible and safe. In this implementation, the constraints include:

1. **State Dynamics Constraint**: This ensures that the predicted states follow the system dynamics described by the kinematic bicycle model.
   $$x[:, t+1] = A x[:, t] + B u[:, t]$$
   where $( A ) and ( B )$ are system matrices derived from the kinematic bicycle model.

2. **Control Input Constraints**: These limit the range of control inputs to realistic and safe values. For example, the steering angle might be limited to avoid excessive turning.

3. **Initial State Constraint**: This sets the first predicted state to be equal to the current state of the system:
   $$x[:, 0] = x_0$$
   where \( $x_0$ \) is the current state vector.

### Error Handling and Robustness

To enhance error handling and robustness in the MPC implementation, the following practices can be adopted:

1. **Validation of Input Parameters**: Before running the optimization, check that all input parameters (e.g., vehicle state, control inputs) are within valid and reasonable bounds. This prevents the solver from encountering infeasible or unrealistic scenarios.

2. **Solver Convergence Checks**: After solving the MPC problem, verify that the solver has successfully converged to an optimal solution. If not, handle the situation gracefully by using fallback strategies or notifying the user.

3. **Constraint Validation**: Ensure that all constraints are correctly defined and remain within their limits during the optimization process. This includes checking that the state dynamics, control inputs, and any other constraints are respected.

4. **Robust Initialization**: Initialize the MPC problem with robust and reasonable initial guesses to improve solver performance and convergence.

### Testing and Performance

Thorough testing and performance profiling are crucial to ensure the reliability and efficiency of the MPC implementation:

1. **Unit Tests**: Develop unit tests for individual functions, particularly those involved in state updates (e.g., `update_state`). This helps in identifying and fixing issues at the function level.

2. **Integration Tests**: Conduct integration tests that simulate the entire MPC loop, including state prediction, optimization, and state updates. This validates the overall functionality of the control system.

3. **Performance Profiling**: Use profiling tools to identify performance bottlenecks in the code. Focus on sections that involve heavy computations, such as the optimization solver. Optimize these sections to reduce computation time and improve real-time performance.

4. **Test Scenarios**: Create a variety of test scenarios to evaluate the performance of the MPC under different conditions. This includes varying initial states, reference trajectories, and external disturbances. Analyze how the MPC adapts to these changes and maintains control.

5. **Optimization of Horizon Length**: Experiment with different prediction horizon lengths to balance between computational complexity and control performance. Longer horizons provide better future insight but require more computation.

By addressing these aspects, the MPC implementation can be made more robust, reliable, and efficient, ultimately improving its real-world applicability and performance.

---

### Extended Use Cases

#### Adaptation to Different Vehicle Types

The Model Predictive Control (MPC) framework is highly versatile and can be adapted to various vehicle types beyond the kinematic bicycle model. Each type of vehicle may have different dynamics, constraints, and control requirements. Here are some examples of how MPC can be extended to other vehicle types:

1. **Dynamic Bicycle Model**: While the kinematic bicycle model is suitable for low-speed applications, the dynamic bicycle model provides a more accurate representation of vehicle dynamics at higher speeds. It accounts for forces and moments acting on the vehicle, making it more appropriate for high-speed maneuvers and stability control.

2. **Trucks and Heavy Vehicles**: For larger vehicles such as trucks and buses, the MPC framework can be adapted to include additional constraints and considerations specific to their dynamics. This includes accounting for longer wheelbases, higher centers of gravity, and different braking and acceleration characteristics.

3. **Articulated Vehicles**: Articulated vehicles, such as semi-trailers, have complex dynamics due to the articulation joint. The MPC framework can be extended to model the interaction between the tractor and trailer, ensuring safe and efficient control of both units.

4. **Autonomous Drones**: MPC can also be applied to the control of unmanned aerial vehicles (UAVs) or drones. The state space and constraints would be different, focusing on 3D motion and incorporating aerodynamics, battery limitations, and collision avoidance.

#### Nonlinear Model Predictive Control (NMPC)

For systems where linear approximations are insufficient, Nonlinear Model Predictive Control (NMPC) provides a more accurate control strategy. NMPC extends the traditional MPC by incorporating nonlinear dynamics directly into the optimization problem. Here are some key aspects of NMPC:

1. **Nonlinear Dynamics**: NMPC can handle the full nonlinear dynamics of the vehicle, providing better performance in scenarios with significant nonlinear behavior, such as sharp turns or rapid acceleration/deceleration.

2. **Advanced Constraints**: NMPC can incorporate more complex constraints that are nonlinear in nature, such as tire friction limits that depend on slip angles and slip ratios.

3. **Improved Accuracy**: By considering the nonlinearities of the system, NMPC can achieve more accurate predictions and control actions, especially in scenarios where linear models fall short.

4. **Increased Computational Demand**: NMPC typically requires more computational resources due to the complexity of solving nonlinear optimization problems. Advanced numerical methods and efficient solvers are crucial for real-time implementation.

#### Potential Applications

1. **Automotive Industry**: NMPC can be used for advanced driver-assistance systems (ADAS) and autonomous driving. It can handle complex maneuvers such as lane changes, obstacle avoidance, and parking in highly dynamic environments.

2. **Robotics**: In robotics, NMPC can be applied to control robotic arms, mobile robots, and humanoid robots. It provides precise control for tasks involving complex dynamics and interactions with the environment.

3. **Aerospace**: For aerospace applications, NMPC can be used in the control of spacecraft, satellites, and UAVs. It enables accurate trajectory planning and control in the presence of nonlinear aerodynamic effects.

4. **Energy Systems**: NMPC can optimize the operation of energy systems such as smart grids, renewable energy sources, and energy storage systems. It accounts for nonlinearities in power generation, consumption, and storage.

By exploring these extended use cases, the MPC framework can be tailored to a wide range of applications, enhancing its versatility and effectiveness in various domains.

---

## Advanced Explaining.

The lower part of the following picture shows in more detail the reference trajectory and the predicted plant outputs. The MPC controller uses its internal prediction model to predict the plant outputs over the prediction horizon _p_. The upper part of the picture shows the control moves planned by the MPC control as well as the first control move, which is the one actually applied to the plant. The control horizon is the number of planned moves, and might be smaller than the prediction horizon.

**MPC signals and horizons**

![The control moves are planned over the control horizon (upper part of the figure) and the first move is applied. The outputs are predicted over the prediction horizon (and, in this case, compared to the reference signal).](https://www.mathworks.com/help/mpc/gs/mpc-intro-plot.png)

When the cost function is quadratic, the plant is linear and without constraints, and the horizon tends to infinity, MPC is equivalent to linear-quadratic Gaussian (LQG) control (or to a linear-quadratic regulator (LQR) control if the plant states are measured and no estimator is used).

In practice, despite the finite horizon, MPC often inherits many useful characteristics of traditional optimal control, such as the ability to naturally handle multi-input multi-output (MIMO) plants, the capability of dealing with time delays (possibly of different durations in different channels), and built-in robustness properties against modeling errors. Nominal stability can also be guaranteed by using specific terminal constraints. Other additional important MPC features are its ability to explicitly handle constraints and the possibility of making use of information on future reference and disturbance signals, when available.

For an introduction on the subject, see first two books in the bibliography sections. For an explanation of the controller internal model and its estimator, see [MPC Prediction Models](https://www.mathworks.com/help/mpc/gs/mpc-modeling.html) and [Controller State Estimation](https://www.mathworks.com/help/mpc/ug/controller-state-estimation.html), respectively. For an overview of the optimization problem, see [Optimization Problem](https://www.mathworks.com/help/mpc/ug/optimization-problem.html). For more information on the solvers, see [QP Solvers](https://www.mathworks.com/help/mpc/ug/qp-solver.html).

Solving a constrained optimal control online at each time step can require substantial computational resources. However in some cases, such as for linear constrained plants, you can precompute and store the control law across the entire state space rather than solve the optimization in real time. This approach is known as explicit MPC.

### MPC Design Workflow

In the simplest case (also known as traditional, or linear, MPC), in which both plant and constraints are linear and the cost function is quadratic, the general workflow to develop an MPC controller includes the following steps.

![MPC design workflow with 7 steps: Specify Plant, Define Signal Type, Create MPC Object, Simulate Closed Loop, Refine Design, Speed up Execution, and Deploy Controller](https://www.mathworks.com/help/mpc/gs/mpc_workflow.png)

1. **Specify plant** — Define the internal plant model that the MPC controller uses to forecast plant behavior across the prediction horizon. Typically, you obtain this plant model by linearizing a nonlinear plant at a given operating point and specifying it as an LTI object, such as [`ss`](https://www.mathworks.com/help/mpc/ref/mpc.ss.html), [`tf`](https://www.mathworks.com/help/control/ref/tf.html), and [`zpk`](https://www.mathworks.com/help/control/ref/zpk.html). You can also identify a plant using System Identification Toolbox™ software. Note that one limitation is that the plant cannot have a direct feedthrough between its control input and any output. For more information on this step, see [Construct Linear Time Invariant Models](https://www.mathworks.com/help/mpc/gs/linear-time-invariant-lti-models.html), [Specify Multi-Input Multi-Output Plants](https://www.mathworks.com/help/mpc/gs/specify-multi-input-multi-output-mimo-plants.html), [Linearize Simulink Models](https://www.mathworks.com/help/mpc/gs/using-simulink-to-develop-lti-models.html), [Linearize Simulink Models Using MPC Designer](https://www.mathworks.com/help/mpc/gs/linearize-simulink-models-using-mpc-designer.html), and [Identify Plant from Data](https://www.mathworks.com/help/mpc/gs/system-identification-toolbox-models.html).
    
2. **Define signal types** — For MPC design purposes, plant signals are usually categorized into different input and output types. You typically use [`setmpcsignals`](https://www.mathworks.com/help/mpc/ref/setmpcsignals.html) to specify, in the plant object defined in the previous step, whether each plant output is measured or unmeasured, and whether each plant input is a manipulated variable (that is, a control input) or a measured or unmeasured disturbance. Alternatively, you can specify signal types in [MPC Designer](https://www.mathworks.com/help/mpc/ref/mpcdesigner-app.html). For more information, see [MPC Signal Types](https://www.mathworks.com/help/mpc/gs/plant-inputs-and-outputs.html).
    
3. **Create MPC object** — After specifying the signal types in the plant object, you create an [`mpc`](https://www.mathworks.com/help/mpc/ref/mpc.html) object in the MATLAB® workspace (or in the [MPC Designer](https://www.mathworks.com/help/mpc/ref/mpcdesigner-app.html)), and specify, in the object, controller parameters such as the sample time, prediction and control horizons, cost function weights, constraints, and disturbance models. The following is an overview of the most important parameters that you need to select.
    
    1. Sample time — A typical starting guess consists of setting the controller sample time so that 10 to 20 samples cover the rise time of the plant.
        
    2. Prediction horizon — The number of future samples over which the controller tries to minimize the cost. It should be long enough to capture the transient response and cover the significant dynamics of the system. A longer horizon increases both performance and computational requirements. A typical prediction horizon is 10 to 20 samples.
        
    3. Control horizon — The number of free control moves that the controller uses to minimize the cost over the prediction horizon. Similarly to the prediction horizon, a longer control horizon increases both performance and computational requirements. A good rule of thumb for the control horizon is to set it from 10% to 20% of the prediction horizon while having a minimum of two to three steps. For more information on sample time and horizon, see [Choose Sample Time and Horizons](https://www.mathworks.com/help/mpc/ug/choosing-sample-time-and-horizons.html).
        
    4. Nominal Values — If your plant is derived from the linearization of a nonlinear model around an operating point, a good practice is to set the nominal values for input, state, state derivative (if nonzero), and output. Doing so allows you to specify constraints on the actual inputs and outputs (instead of doing so on the deviations from their nominal values), and allows you to simulate the closed loop and visualize signals more easily when using Simulink® or the [`sim`](https://www.mathworks.com/help/mpc/ref/mpc.sim.html) command.
        
    5. Scale factors — Good practice is to specify scale factors for each plant input and output, especially when their range and magnitude is very different. Appropriate scale factors improve the numerical condition of the underlying optimization problem and make weight tuning easier. A good recommendation is to set a scale factor approximatively equal to the span (the difference between the maximum and minimum value in engineering units) of the related signal. For more information, see [Specify Scale Factors](https://www.mathworks.com/help/mpc/ug/scale-factors.html).
        
    6. Constraints — Constraints typically reflect physical limits. You can specify constraints as either hard (cannot be violated in the optimization) or soft (can be violated to a small extent). A good recommendation is to set hard constraints, if necessary, on the inputs or their rate of change, while setting output constraints, if necessary, as soft. Setting hard constraints on both input and outputs can lead to infeasibility and is in general not recommended. For more information, see [Specify Constraints](https://www.mathworks.com/help/mpc/ug/specifying-constraints.html).
        
    7. Weights — You can prioritize the performance goals of your controller by adjusting the cost function tuning weights. Typically, larger output weights provide aggressive reference tracking performance, while larger weights on the manipulated variable rates promote smoother control moves that improve robustness. For more information, see [Tune Weights](https://www.mathworks.com/help/mpc/ug/tuning-weights.html).
        
    8. Disturbance and noise models — The internal prediction model that the controller uses to calculate the control action typically consists of the plant model augmented with models for disturbances and measurement noise affecting the plant. Disturbance models specify the dynamic characteristics of the unmeasured disturbances on the inputs and outputs, respectively, so they can be better rejected. By default, these disturbance models are assumed to be integrators (therefore allowing the controller to reject step-like disturbances) unless you specify them otherwise. Measurement noise is typically assumed to be white. For more information on plant and disturbance models see [MPC Prediction Models](https://www.mathworks.com/help/mpc/gs/mpc-modeling.html), and [Adjust Disturbance and Noise Models](https://www.mathworks.com/help/mpc/ug/adjusting-disturbance-and-noise-models.html).
        
    
    After creating the [`mpc`](https://www.mathworks.com/help/mpc/ref/mpc.html) object, good practice is to use functions such as [`cloffset`](https://www.mathworks.com/help/mpc/ref/mpc.cloffset.html) to calculate the closed loop steady state output sensitivities, therefore checking whether the controller can reject constant output disturbances. The more general [`review`](https://www.mathworks.com/help/mpc/ref/mpc.review.html) also inspects the object for potential problems. To perform a deeper sensitivity and robustness analysis for the time frames in which you expect no constraint to be active, you can also convert the unconstrained controller to an LTI system object using [`ss`](https://www.mathworks.com/help/mpc/ref/mpc.ss.html), [`zpk`](https://www.mathworks.com/help/mpc/ref/mpc.zpk.html), or [`tf`](https://www.mathworks.com/help/mpc/ref/mpc.tf.html). For related examples, see [Review Model Predictive Controller for Stability and Robustness Issues](https://www.mathworks.com/help/mpc/ug/reviewing-model-predictive-controller-design-for-potential-stability-and-robustness-issues.html), [Test MPC Controller Robustness Using MPC Designer](https://www.mathworks.com/help/mpc/ug/robustness-testing.html), [Compute Steady-State Output Sensitivity Gain](https://www.mathworks.com/help/mpc/ug/compute-steady-state-gain.html), and [Extract Controller](https://www.mathworks.com/help/mpc/ug/extract-controller.html).
    
    Note that many of the recommended parameter choices are incorporated in the default values of the [`mpc`](https://www.mathworks.com/help/mpc/ref/mpc.html) object; however, since each of these parameter is normally the result of several problem-dependent trade offs, you have to select the parameters that make sense for your particular plant and requirements.
    
4. **Simulate closed loop** — After you create an MPC controller, you typically evaluate the performance of your controller by simulating it in closed loop with your plant using one of the following options.
    
    - Using MATLAB, you can simulate the closed loop using [`sim`](https://www.mathworks.com/help/mpc/ref/mpc.sim.html) (more convenient for linear plant models) or [`mpcmove`](https://www.mathworks.com/help/mpc/ref/mpc.mpcmove.html) (more flexible, allowing for more general discrete time plants or disturbance signals and for a custom state estimator).
        
    - Using Simulink, you can use the [MPC Controller](https://www.mathworks.com/help/mpc/ref/mpccontroller.html) block (which takes your [`mpc`](https://www.mathworks.com/help/mpc/ref/mpc.html) object as a parameter) in closed loop with your plant model built in Simulink. This option allows for the greatest flexibility in simulating more complex systems and for easy generation of production code from your controller.
        
    - Using [MPC Designer](https://www.mathworks.com/help/mpc/ref/mpcdesigner-app.html), you can simulate the linear closed loop response while at the same time tuning the controller parameters.
        
    
    Note that any of these options allows you to also simulate model mismatches (cases in which the actual plant is slightly different from the internal plant model that the controller uses for prediction). For a related example, see [Simulating MPC Controller with Plant Model Mismatch](https://www.mathworks.com/help/mpc/ug/simulate-model-predictive-controllers-with-plant-model-mismatch.html). When reference and measured disturbances are known ahead of time, MPC can use this information (also known as look-ahead, or previewing) to improve the controller performance. See [Signal Previewing](https://www.mathworks.com/help/mpc/ug/signal-previewing.html) for more information and [Improving Control Performance with Look-Ahead (Previewing)](https://www.mathworks.com/help/mpc/ug/improve-control-performance-with-look-ahead.html) for a related example. Similarly, you can specify tuning weights and constraints that vary over the prediction horizon. For related examples, see [Update Constraints at Run Time](https://www.mathworks.com/help/mpc/ug/run-time-constraint-updating.html), [Vary Input and Output Bounds at Run Time](https://www.mathworks.com/help/mpc/ug/vary-input-and-output-bounds-at-run-time.html), [Tune Weights at Run Time](https://www.mathworks.com/help/mpc/ug/run-time-weight-tuning.html), and [Adjust Horizons at Run Time](https://www.mathworks.com/help/mpc/ug/adjust-horizons-at-run-time.html).
    
5. **Refine design** — After an initial evaluation of the closed loop you typically need to refine the design by adjusting the controller parameters and evaluating different simulation scenarios. In addition to the parameters described in step 3, you can consider:
    
    - Using manipulated variable blocking. For more information, see [Manipulated Variable Blocking](https://www.mathworks.com/help/mpc/ug/manipulated-variable-blocking.html).
        
    - For over-actuated systems, setting reference targets for the manipulated variables. For a related example, see [Setting Targets for Manipulated Variables](https://www.mathworks.com/help/mpc/ug/setting-targets-for-manipulated-variables.html).
        
    - Tuning the gains of the Kalman state estimator (or designing a custom state estimator). For more information and related examples, see [Controller State Estimation](https://www.mathworks.com/help/mpc/ug/controller-state-estimation.html), [Custom State Estimation](https://www.mathworks.com/help/mpc/ug/custom-state-estimation.html), and [Implement Custom State Estimator Equivalent to Built-In Kalman Filter](https://www.mathworks.com/help/mpc/ug/design-estimator-equivalent-to-mpc-built-in-kf.html).
        
    - Specifying terminal constraints. For more information and a related example, see [Terminal Weights and Constraints](https://www.mathworks.com/help/mpc/ug/terminal-weights-and-constraints.html) and [Provide LQR Performance Using Terminal Penalty Weights](https://www.mathworks.com/help/mpc/ug/using-terminal-penalty-to-provide-lqr-performance.html).
        
    - Specifying custom constraints. For related examples, see [Constraints on Linear Combinations of Inputs and Outputs](https://www.mathworks.com/help/mpc/ug/constraints-on-linear-combinations-of-inputs-and-outputs.html) and [Use Custom Constraints in Blending Process](https://www.mathworks.com/help/mpc/ug/custom-constraints-in-a-blending-process.html).
        
    - Specifying off-diagonal cost function weights. For an example, see [Specifying Alternative Cost Function with Off-Diagonal Weight Matrices](https://www.mathworks.com/help/mpc/ug/specifying-alternative-cost-function-with-off-diagonal-weight-matrices.html).
        
    
6. **Speed up execution** — See [MPC Controller Deployment](https://www.mathworks.com/help/mpc/gs/what-is-mpc.html#mw_d02c2083-dc4b-4e4f-8226-3953e7b2c6cd).
    
7. **Deploy controller** — See [MPC Controller Deployment](https://www.mathworks.com/help/mpc/gs/what-is-mpc.html#mw_d02c2083-dc4b-4e4f-8226-3953e7b2c6cd).
    

### Control Nonlinear and Time-Varying Plants

Often the plant to be controlled can be accurately approximated by a linear plant only locally, around a given operating point. This approximation might no longer be accurate as time passes and the plant operating point changes.

You can use several approaches to deal with these cases, from the simpler to more general and complicated.

1. Adaptive MPC — If the order (and the number of time delays) of the plant does not change, you can design a single MPC controller (for example for the initial operating point), and then at run-time you can update the controller prediction model at each time step (while the controller still assumes that the prediction model stays constant in the future, across its prediction horizon).
    
    Note that while this approach is the simplest, it requires you to continuously (that is, at each time step) calculate the linearized plant that has to be supplied to the controller. You can do so in three main ways.
    
    1. If you have a reliable plant model, you can extract the local linear plant online by linearizing the equations, assuming this process is not too computationally expensive. If you have simple symbolic equations for your plant model, you might be able to derive, offline, a symbolic expression of the linearized plant matrices at any given operating condition. Online, you can then calculate these matrices and supply them to the adaptive MPC controller without having to perform a numerical linearization at each time step. For an example using this strategy, see [Adaptive MPC Control of Nonlinear Chemical Reactor Using Successive Linearization](https://www.mathworks.com/help/mpc/ug/adaptive-mpc-control-of-nonlinear-chemical-reactor-using-successive-linearization.html).
        
    2. Alternatively, you can extract an array of linearized plant models offline, covering the relevant regions of the state-input space, and then online you can use a linear parameter-varying (LPV) plant that obtains, by interpolation, the linear plant at the current operating point. For an example using this strategy, see [Adaptive MPC Control of Nonlinear Chemical Reactor Using Linear Parameter-Varying System](https://www.mathworks.com/help/mpc/ug/adaptive-mpc-control-of-nonlinear-chemical-reactor-using-linear-parameter-varying-system.html).
        
    3. If the plant is not accurately represented by a mathematical model, but you can assume a known structure with some estimates of its parameters, stability, and a minimal amount of input noise, you can use the past plant inputs and outputs to estimate a model of the plant online, although this can be somewhat computationally intensive. For an example using this strategy, see [Adaptive MPC Control of Nonlinear Chemical Reactor Using Online Model Estimation](https://www.mathworks.com/help/mpc/ug/adaptive-mpc-control-of-nonlinear-chemical-reactor-using-online-model-estimation.html).
        
    
    This approach requires an [`mpc`](https://www.mathworks.com/help/mpc/ref/mpc.html) object and either the [`mpcmoveAdaptive`](https://www.mathworks.com/help/mpc/ref/mpc.mpcmoveadaptive.html) function or the [Adaptive MPC Controller](https://www.mathworks.com/help/mpc/ref/adaptivempccontroller.html) block. For more information, see [Adaptive MPC](https://www.mathworks.com/help/mpc/ug/adaptive-mpc.html) and [Model Updating Strategy](https://www.mathworks.com/help/mpc/ug/model-updating-strategy.html).
    
2. Linear Time Varying MPC — This approach is a kind of adaptive MPC in which the controller knows in advance how its internal plant model changes in the future, and therefore uses this information when calculating the optimal control across the prediction horizon. Here, at every time step, you supply to the controller not only the current plant model but also the plant models for all the future steps, across the whole prediction horizon. To calculate the plant models for the future steps, you can use the manipulated variables and plant states predicted by the MPC controller at each step as operating points around which a nonlinear plant model can be linearized.
    
    This approach is particularly useful when the plant model changes considerably (but predictably) within the prediction horizon. It requires an [`mpc`](https://www.mathworks.com/help/mpc/ref/mpc.html) object and using [`mpcmoveAdaptive`](https://www.mathworks.com/help/mpc/ref/mpc.mpcmoveadaptive.html) or the [Adaptive MPC Controller](https://www.mathworks.com/help/mpc/ref/adaptivempccontroller.html) block. For more information, see [Time-Varying MPC](https://www.mathworks.com/help/mpc/ug/time-varying-mpc.html).
    
3. Gain-Scheduled MPC — In this approach you design multiple MPC controllers offline, one for each relevant operating point. Then, online, you switch the active controller as the plant operating point changes. While switching the controller is computationally simple, this approach requires more online memory (and in general more design effort) than adaptive MPC. It should be reserved for cases in which the linearized plant models have different orders or time delays (and the switching variable changes slowly, with respect to the plant dynamics). To use gain-scheduled MPC. you create an array of [`mpc`](https://www.mathworks.com/help/mpc/ref/mpc.html) objects and then use the [`mpcmoveMultiple`](https://www.mathworks.com/help/mpc/ref/mpcmovemultiple.html) function or the [Multiple MPC Controllers](https://www.mathworks.com/help/mpc/ref/multiplempccontrollers.html) block for simulation. For more information, see [Gain-Scheduled MPC](https://www.mathworks.com/help/mpc/ug/gain-scheduling-mpc.html). For an example, see [Gain-Scheduled MPC Control of Nonlinear Chemical Reactor](https://www.mathworks.com/help/mpc/ug/gain-scheduling-mpc-control-of-nonlinear-chemical-reactor.html).
    
4. Nonlinear MPC — You can use this strategy to control highly nonlinear plants when all the previous approaches are unsuitable, or when you need to use nonlinear constraints or non-quadratic cost functions. This approach is more computationally intensive than the previous ones, and it also requires you to design an implement a nonlinear state estimator if the plant state is not completely available. Two nonlinear MPC approaches are available.
    
    1. Multistage Nonlinear MPC — For a multistage MPC controller, each future step in the horizon (stage) has its own decision variables and parameters, as well as its own nonlinear cost and constraints. Crucially, cost and constraint functions at a specific stage are functions only of the decision variables and parameters at that stage. While specifying multiple costs and constraint functions can require more design time, it also allows for an efficient formulation of the underlying optimization problem and a smaller data structure, which significantly reduces computation times compared to generic nonlinear MPC. Use this approach if your nonlinear MPC problem has cost and constraint functions that do not involve cross-stage terms, as is often the case. To use multistage nonlinear MPC you need to create an [`nlmpcMultistage`](https://www.mathworks.com/help/mpc/ref/nlmpcmultistage.html) object, and then use the [`nlmpcmove`](https://www.mathworks.com/help/mpc/ref/nlmpc.nlmpcmove.html) function or the [Multistage Nonlinear MPC Controller](https://www.mathworks.com/help/mpc/ref/multistagenonlinearmpccontroller.html) block for simulation. For more information, see [Multistage Nonlinear MPC](https://www.mathworks.com/help/mpc/ug/nonlinear-mpc.html#mw_a1f2282f-5bb3-4f93-a539-117d031d44f7).
        
    2. Generic Nonlinear MPC — This method is the most general, and computationally expensive, form of MPC. As it explicitly provides standard weights and linear bounds settings, it can be a good starting point for a design where the only nonlinearity comes from the plant model. Furthermore, you can use the `RunAsLinearMPC` option in the [`nlmpc`](https://www.mathworks.com/help/mpc/ref/nlmpc.html) object to evaluate whether linear, time varying, or adaptive MPC can achieve the same performance. If so, use these design options, and possibly evaluate gain scheduled MPC; otherwise, consider multistage nonlinear MPC. Use generic nonlinear MPC only as an initial design, or when all the previous design options are not viable. To use generic nonlinear MPC, you need to create an [`nlmpc`](https://www.mathworks.com/help/mpc/ref/nlmpc.html) object, and then use the [`nlmpcmove`](https://www.mathworks.com/help/mpc/ref/nlmpc.nlmpcmove.html) function or the [Nonlinear MPC Controller](https://www.mathworks.com/help/mpc/ref/nonlinearmpccontroller.html) block for simulation. For more information, see [Generic Nonlinear MPC](https://www.mathworks.com/help/mpc/ug/nonlinear-mpc.html#mw_573777ed-9fd6-4973-9aee-bf91538d133a).
        
    

### MPC Controller Deployment

When you are satisfied with the simulation performance of your controller design, you typically look for ways to speed up the execution, in an effort to both optimize the design for future simulations and to meet the stricter computational requirements of embedded applications.

You can use several strategies to improve the computational performance of MPC controllers.

1. Try to increase the sample time — The sampling frequency must be high enough to cover the significant bandwidth of the system. However, if the sample time is too small, not only do you reduce the available computation time for the controller but you must also use a larger prediction horizon to cover the system response, which increases computational requirements.
    
2. Try to shorten prediction and control horizons — Since both horizons directly impact the total number of decision variables and constraints in the resulting optimization problem, they heavily affect both memory usage and the number of required calculations. Therefore, check whether you can obtain similar tracking responses and robustness to uncertainties with shorter horizons. Note that sample time plays a role too. The sampling frequency needs to be high enough (equivalently the sample time small enough) to cover the significant bandwidth of the system. However, if the sample time is too small, not only you have a shorter available execution time on the hardware, but you also need a larger number of prediction steps to cover the system response, which results in a more computationally expensive optimization problem to be solved at each time step.
    
3. Use varying parameters only when needed — Normally Model Predictive Control Toolbox™ allows you to vary some parameters (such as weights or constraints coefficients) at run-rime. While this capability is useful in some cases, it considerably increases the complexity of the software. Therefore, unless specifically needed, for deployment, consider explicitly specifying such parameters as constants, thus preventing the possibility of changing them online. For related examples, see [Update Constraints at Run Time](https://www.mathworks.com/help/mpc/ug/run-time-constraint-updating.html), [Vary Input and Output Bounds at Run Time](https://www.mathworks.com/help/mpc/ug/vary-input-and-output-bounds-at-run-time.html), [Tune Weights at Run Time](https://www.mathworks.com/help/mpc/ug/run-time-weight-tuning.html), and [Adjust Horizons at Run Time](https://www.mathworks.com/help/mpc/ug/adjust-horizons-at-run-time.html).
    
4. Limit the maximum number of iterations that your controller can use to solve the quadratic optimization problem, and configure it to use the current suboptimal solution when the maximum number of iterations is reached. Using a suboptimal solution shortens the time needed by the controller to calculate the control action, and in some cases it does not significantly decrease performance. In any case, since the number of iterations can change dramatically from one control interval to the next, for real time applications, it is recommended to limit the maximum number of iterations. Doing so helps ensuring that the worst-case execution time does not exceed the total computation time allowed on the hardware platform, which is determined by the controller sample time. For a related example, see [Use Suboptimal Solution in Fast MPC Applications](https://www.mathworks.com/help/mpc/ug/simulate-mpc-controller-using-suboptimal-solution.html).
    
5. Tune the solver and its options — The default Model Predictive Control Toolbox solver is a "dense," "active set" solver based on the [KWIK algorithm](https://www.sciencedirect.com/science/article/abs/pii/0098135494E00014), and it typically performs well in many cases. However, if the total number of manipulated variables, outputs, and constraints across the whole horizon is large, you might consider using an interior point solver. For resource-constrained embedded applications, consider the ADMM solver. If the internal plant is highly open-loop unstable, consider using a sparse solver. For an overview of the optimization problem, see [Optimization Problem](https://www.mathworks.com/help/mpc/ug/optimization-problem.html). For more information on the solvers, see [QP Solvers](https://www.mathworks.com/help/mpc/ug/qp-solver.html) and [Configure Optimization Solver for Nonlinear MPC](https://www.mathworks.com/help/mpc/ug/configure-optimization-solver-for-nonlinear-mpc.html). For related examples, see [Simulate MPC Controller with a Custom QP Solver](https://www.mathworks.com/help/mpc/ug/simulate-mpc-controller-with-a-custom-qp-solver.html) and [Optimizing Tuberculosis Treatment Using Nonlinear MPC with a Custom Solver](https://www.mathworks.com/help/mpc/ug/optimize-tuberculosis-treatment-using-nonlinear-mpc-with-custom-solver.html).
    

For applications with extremely fast sample time, consider using explicit MPC. It can be proven that the solution to the linear MPC problem (quadratic cost function, linear plant, and constraints) is piecewise affine (PWA) on polyhedra. In other words, the constraints divide the state space into polyhedral "critical" regions in which the optimal control action is an affine (linear plus a constant) function of the state. The idea behind explicit MPC is to precalculate, offline and once for all, these functions of the state for every region. These functions can then be stored in your controller. At run time, the controller then selects and applies the appropriate state feedback law, depending on the critical region that the current operating point is in. Since explicit MPC controllers do not solve an optimization problem online, they require much fewer computations and are therefore useful for applications requiring small sample times. On the other hand, they also have a much larger memory footprint. Indeed, excessive memory requirements can render this approach no longer viable for medium to large problems. Also, since explicit MPC pre-computes the controller offline, it does not support runtime updates of parameters such as weights, constraints or horizons.

To use explicit MPC, you need to generate an [`explicitMPC`](https://www.mathworks.com/help/mpc/ref/explicitmpc.html) object from an existing [`mpc`](https://www.mathworks.com/help/mpc/ref/mpc.html) object and then use the [`mpcmoveExplicit`](https://www.mathworks.com/help/mpc/ref/explicitmpc.mpcmoveexplicit.html) function or the [Explicit MPC Controller](https://www.mathworks.com/help/mpc/ref/explicitmpccontroller.html) block for simulation. For more information, see [Explicit MPC](https://www.mathworks.com/help/mpc/ug/explicit-mpc.html).

A final option to consider to improve the computational performance of both implicit and explicit MPC is to simplify the problem. Some parameters, such as the number of constraints and the number of state variables, greatly increase the complexity of the resulting optimization problem. Therefore, if the previous options are not satisfying, consider returning these parameters (and potentially using a simpler lower-fidelity prediction model) to simplify the problem.

Once you are satisfied with the computational performance of your design, you can generate code for deployment to real-time applications from MATLAB or Simulink. For more information, see [Generate Code and Deploy Controller to Real-Time Targets](https://www.mathworks.com/help/mpc/ug/generate-code-and-deploy-controller-to-real-time-targets.html).


---
## Conclusion

This project provides an excellent introduction and practical example of Model Predictive Control for vehicle speed and steering. The detailed explanations and well-organized code make it accessible to both beginners and experienced practitioners. By enhancing documentation, improving error handling, and incorporating thorough testing, the overall robustness and utility of the implementation can be further improved.

## References

- [MPC Overview](https://en.wikipedia.org/wiki/Model_predictive_control)
- [Kinematic Bicycle Model](https://www.ri.cmu.edu/pub_files/2015/7/Rajamani_Kinematic_Bicycle_Model.pdf)
- [cvxpy Documentation](https://www.cvxpy.org/)

---
