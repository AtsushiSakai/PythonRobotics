import random

import conftest
from PathPlanning.BatchInformedRRTStar import batch_informed_rrt_star as m


def test_1():
    m.show_animation = False
    random.seed(12345)
    m.main(maxIter=10)


def test_obstacles_are_drawn_with_their_radius_in_data_units(monkeypatch):
    # Obstacles must be drawn as circles whose radius matches the
    # collision radius, independent of the figure size (issue #1396).
    monkeypatch.setattr(m.plt, "pause", lambda interval: None)
    obstacle_list = [(5.0, 5.0, 1.0), (3.0, 6.0, 2.0)]
    planner = m.BITStar(start=[0.0, 0.0], goal=[10.0, 10.0], obstacleList=obstacle_list, randArea=[-2.0, 15.0])

    m.plt.figure(figsize=(12, 12))
    planner.draw_graph(samples=[])

    circles = [patch for patch in m.plt.gca().patches
               if isinstance(patch, m.plt.Circle)]
    m.plt.close("all")
    assert [(circle.center, circle.radius) for circle in circles] == [
        ((ox, oy), size) for (ox, oy, size) in obstacle_list]


if __name__ == '__main__':
    conftest.run_this_test(__file__)
