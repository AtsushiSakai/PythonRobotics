import conftest  # Add root path to sys.path
from PathPlanning.RRTStarDubins import rrt_star_dubins as m


def test1():
    m.show_animation = False
    m.main()


def test_obstacles_are_drawn_with_their_radius_in_data_units(monkeypatch):
    # Obstacles must be drawn as circles whose radius matches the
    # collision radius, independent of the figure size (issue #1396).
    monkeypatch.setattr(m.plt, "pause", lambda interval: None)
    obstacle_list = [(5.0, 5.0, 1.0), (3.0, 6.0, 2.0)]
    planner = m.RRTStarDubins([0.0, 0.0, 0.0], [10.0, 10.0, 0.0], obstacle_list, [-2.0, 15.0])
    planner.node_list = [planner.start]

    m.plt.figure(figsize=(12, 12))
    planner.draw_graph()

    circles = [patch for patch in m.plt.gca().patches
               if isinstance(patch, m.plt.Circle)]
    m.plt.close("all")
    assert [(circle.center, circle.radius) for circle in circles] == [
        ((ox, oy), size) for (ox, oy, size) in obstacle_list]


if __name__ == '__main__':
    conftest.run_this_test(__file__)
