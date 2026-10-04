import conftest
import numpy as np

from PathPlanning.DynamicWindowApproach import dynamic_window_approach as m


def test_main1():
    m.show_animation = False
    m.main(gx=1.0, gy=1.0)


def test_main2():
    m.show_animation = False
    m.main(gx=1.0, gy=1.0, robot_type=m.RobotType.rectangle)


def test_stuck_main():
    m.show_animation = False
    # adjust cost
    m.config.to_goal_cost_gain = 0.2
    m.config.obstacle_cost_gain = 2.0
    # obstacles and goals for stuck condition
    m.config.ob = -1 * np.array([[-1.0, -1.0],
                                 [0.0, 2.0],
                                 [2.0, 6.0],
                                 [2.0, 8.0],
                                 [3.0, 9.27],
                                 [3.79, 9.39],
                                 [7.25, 8.97],
                                 [7.0, 2.0],
                                 [3.0, 4.0],
                                 [6.0, 5.0],
                                 [3.5, 5.8],
                                 [6.0, 9.0],
                                 [8.8, 9.0],
                                 [5.0, 9.0],
                                 [7.5, 3.0],
                                 [9.0, 8.0],
                                 [5.8, 4.4],
                                 [12.0, 12.0],
                                 [3.0, 2.0],
                                 [13.0, 13.0]
                                 ])
    m.main(gx=-5.0, gy=-7.0)


def _rectangle_config():
    config = m.Config()
    config.robot_type = m.RobotType.rectangle
    config.robot_length = 2.0
    config.robot_width = 0.2
    return config


def _scalar_obstacle_cost(trajectory, obstacles, config):
    # Independent per-pose geometry: project each obstacle onto the robot's
    # longitudinal and lateral axes at that pose.
    import math

    minimum_distance = float("inf")
    for pose in trajectory:
        cosine, sine = math.cos(pose[2]), math.sin(pose[2])
        for obstacle in obstacles:
            dx, dy = obstacle[0] - pose[0], obstacle[1] - pose[1]
            longitudinal = cosine * dx + sine * dy
            lateral = -sine * dx + cosine * dy
            if (abs(longitudinal) <= config.robot_length / 2 and
                    abs(lateral) <= config.robot_width / 2):
                return float("inf")
            minimum_distance = min(minimum_distance, math.hypot(dx, dy))
    return 1.0 / minimum_distance


def test_rectangular_cost_keeps_translation_and_heading_paired():
    config = _rectangle_config()
    trajectory = np.array([[0., 0., 0., 0., 0.],
                           [10., 10., np.pi / 2, 0., 0.]])
    obstacles = np.array([[0., 0.6]])
    # The obstacle is outside both footprints. Applying the second pose's yaw
    # to the first pose's offset invents a collision.
    np.testing.assert_allclose(m.calc_obstacle_cost(trajectory, obstacles, config),
                               1.0 / 0.6)


def test_rectangular_cost_matches_scalar_geometry():
    config = _rectangle_config()
    rng = np.random.default_rng(712)
    for n_steps in [1, 2, 7, 20]:
        for n_obstacles in [1, 3, 15]:
            for _ in range(10):
                trajectory = np.zeros((n_steps, 5))
                trajectory[:, :2] = rng.uniform(-5., 5., (n_steps, 2))
                trajectory[:, 2] = rng.uniform(-np.pi, np.pi, n_steps)
                obstacles = rng.uniform(-5., 5., (n_obstacles, 2))
                actual = m.calc_obstacle_cost(trajectory, obstacles, config)
                expected = _scalar_obstacle_cost(trajectory, obstacles, config)
                np.testing.assert_allclose(actual, expected)


def test_rectangular_cost_preserves_true_contact():
    config = _rectangle_config()
    trajectory = np.array([[0., 0., 0., 0., 0.],
                           [10., 10., np.pi / 2, 0., 0.]])
    for obstacles in [np.array([[0.5, 0.05]]), np.array([[10., 10.5]])]:
        assert np.isinf(m.calc_obstacle_cost(trajectory, obstacles, config))


def test_rectangular_cost_is_invariant_under_world_transform():
    config = _rectangle_config()
    trajectory = np.array([[0., 0., 0., 0., 0.],
                           [10., 10., np.pi / 2, 0., 0.]])
    obstacles = np.array([[0., 0.6]])
    angle = 0.7
    rotation = np.array([[np.cos(angle), -np.sin(angle)],
                         [np.sin(angle), np.cos(angle)]])
    transformed_trajectory = trajectory.copy()
    transformed_trajectory[:, :2] = trajectory[:, :2] @ rotation.T + [3., -4.]
    transformed_trajectory[:, 2] += angle
    transformed_obstacles = obstacles @ rotation.T + [3., -4.]
    actual = m.calc_obstacle_cost(transformed_trajectory, transformed_obstacles, config)
    np.testing.assert_allclose(actual, 1.0 / 0.6)


if __name__ == '__main__':
    conftest.run_this_test(__file__)
