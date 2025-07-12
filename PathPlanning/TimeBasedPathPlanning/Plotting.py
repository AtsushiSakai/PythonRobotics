import numpy as np
import matplotlib.pyplot as plt
from matplotlib.backend_bases import KeyEvent
from PathPlanning.TimeBasedPathPlanning.GridWithDynamicObstacles import (
    Grid,
    Position,
)
from PathPlanning.TimeBasedPathPlanning.BaseClasses import StartAndGoal
from PathPlanning.TimeBasedPathPlanning.Node import NodePath

'''
Plot a single agent path.
'''
def PlotNodePath(grid: Grid, start: Position, goal: Position, path: NodePath):
    fig = plt.figure(figsize=(10, 7))
    ax = fig.add_subplot(
        autoscale_on=False,
        xlim=(0, grid.grid_size[0] - 1),
        ylim=(0, grid.grid_size[1] - 1),
    )
    ax.set_aspect("equal")
    ax.grid()
    ax.set_xticks(np.arange(0, grid.grid_size[0], 1))
    ax.set_yticks(np.arange(0, grid.grid_size[1], 1))

    (start_and_goal,) = ax.plot([], [], "mD", ms=15, label="Start and Goal")
    start_and_goal.set_data([start.x, goal.x], [start.y, goal.y])
    (obs_points,) = ax.plot([], [], "ro", ms=15, label="Obstacles")
    (path_points,) = ax.plot([], [], "bo", ms=10, label="Path Found")
    ax.legend(bbox_to_anchor=(1.05, 1))

    # for stopping simulation with the esc key.
    plt.gcf().canvas.mpl_connect(
        "key_release_event",
        lambda event: [exit(0) if event.key == "escape" else None]
        if isinstance(event, KeyEvent) else None
    )

    for i in range(0, path.goal_reached_time()):
        obs_positions = grid.get_obstacle_positions_at_time(i)
        obs_points.set_data(obs_positions[0], obs_positions[1])
        path_position = path.get_position(i)
        if not path_position:
            raise Exception(f"Path position not found for time {i}.")

        path_points.set_data([path_position.x], [path_position.y])
        plt.pause(0.2)
    plt.show()

'''
Plot a series of agent paths.
'''
def PlotNodePaths(grid: Grid, start_and_goals: list[StartAndGoal], paths: list[NodePath]):
    fig = plt.figure(figsize=(10, 7))

    ax = fig.add_subplot(
        autoscale_on=False,
        xlim=(0, grid.grid_size[0] - 1),
        ylim=(0, grid.grid_size[1] - 1),
    )
    ax.set_aspect("equal")
    ax.grid()
    ax.set_xticks(np.arange(0, grid.grid_size[0], 1))
    ax.set_yticks(np.arange(0, grid.grid_size[1], 1))

    # Plot start and goal positions for each agent
    colors = [] # generated randomly in loop
    markers = ['D', 's', '^', 'o', 'p']  # Different markers for visual distinction

    # Create plots for start and goal positions
    start_and_goal_plots = []
    for i, path in enumerate(paths):
        marker_idx = i % len(markers)
        agent_id = start_and_goals[i].index
        start = start_and_goals[i].start
        goal = start_and_goals[i].goal
        
        color = np.random.rand(3,)
        colors.append(color)
        sg_plot, = ax.plot([], [], markers[marker_idx], c=color, ms=15, 
                            label=f"Agent {agent_id} Start/Goal")
        sg_plot.set_data([start.x, goal.x], [start.y, goal.y])
        start_and_goal_plots.append(sg_plot)

    # Plot for obstacles
    (obs_points,) = ax.plot([], [], "ro", ms=15, label="Obstacles")

    # Create plots for each agent's path
    path_plots = []
    for i, path in enumerate(paths):
        agent_id = start_and_goals[i].index
        path_plot, = ax.plot([], [], "o", c=colors[i], ms=10, 
                            label=f"Agent {agent_id} Path")
        path_plots.append(path_plot)

    ax.legend(bbox_to_anchor=(1.05, 1))

    # For stopping simulation with the esc key
    plt.gcf().canvas.mpl_connect(
        "key_release_event",
        lambda event: [exit(0) if event.key == "escape" else None]
        if isinstance(event, KeyEvent) else None
    )

    # Find the maximum time across all paths
    max_time = max(path.goal_reached_time() for path in paths)

    # Animation loop
    for i in range(0, max_time + 1):
        # Update obstacle positions
        obs_positions = grid.get_obstacle_positions_at_time(i)
        obs_points.set_data(obs_positions[0], obs_positions[1])
        
        # Update each agent's position
        for (j, path) in enumerate(paths):
            path_positions = []
            if i <= path.goal_reached_time():
                res = path.get_position(i)
                if not res:
                    print(path)
                    print(i)
                path_position = path.get_position(i)
                if not path_position:
                    raise Exception(f"Path position not found for time {i}.")

                # Verify position is valid
                assert not path_position in obs_positions
                assert not path_position in path_positions
                path_positions.append(path_position)

                path_plots[j].set_data([path_position.x], [path_position.y])
        
        plt.pause(0.2)

    plt.show()