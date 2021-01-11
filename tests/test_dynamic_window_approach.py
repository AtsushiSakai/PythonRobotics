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


if __name__ == '__main__':
    conftest.run_this_test(__file__)
