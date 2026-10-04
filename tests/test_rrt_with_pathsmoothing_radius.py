import conftest
import math

from PathPlanning.RRT import rrt_with_pathsmoothing as rrt_module

def test_smoothed_path_safety():
    # Define test environment
    obstacle_list = [
        (5, 5, 1.0),
        (3, 6, 2.0),
        (3, 8, 2.0),
        (3, 10, 2.0),
        (7, 5, 2.0),
        (9, 5, 2.0)
    ]
    robot_radius = 0.5

    # Disable animation for testing
    rrt_module.show_animation = False

    # Create RRT planner
    rrt = rrt_module.RRT(
        start=[0, 0],
        goal=[6, 10],
        rand_area=[-2, 15],
        obstacle_list=obstacle_list,
        robot_radius=robot_radius
    )

    # Run RRT
    path = rrt.planning(animation=False)

    # Smooth the path
    smoothed = rrt_module.path_smoothing(path, max_iter=1000,
                                         obstacle_list=obstacle_list,
                                         robot_radius=robot_radius)

    # Check if all points on the smoothed path are safely distant from obstacles
    for x, y in smoothed:
        for ox, oy, obs_radius in obstacle_list:
            d = math.hypot(x - ox, y - oy)
            min_safe_dist = obs_radius + robot_radius
            assert d > min_safe_dist, \
                f"Point ({x:.2f}, {y:.2f}) too close to obstacle at ({ox}, {oy})"


if __name__ == '__main__':
    conftest.run_this_test(__file__)
