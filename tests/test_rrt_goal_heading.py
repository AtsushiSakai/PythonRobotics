import math

import pytest

import conftest  # Add root path to sys.path
from PathPlanning.RRTDubins.rrt_dubins import RRTDubins
from PathPlanning.RRTStarDubins.rrt_star_dubins import RRTStarDubins
from PathPlanning.ClosedLoopRRTStar import closed_loop_rrt_star_car as closed_loop


@pytest.mark.parametrize("planner_class", [RRTDubins, RRTStarDubins])
@pytest.mark.parametrize(
    "candidate, goal",
    [(math.pi - 0.005, -math.pi + 0.005),
     (-math.pi + 0.005, math.pi - 0.005),
     (0.2 + 2 * math.pi, 0.2),
     (-0.2 - 2 * math.pi, -0.2)],
)
def test_dubins_goal_accepts_equivalent_headings(planner_class, candidate, goal):
    planner = planner_class(
        start=[-1.0, 0.0, 0.0], goal=[0.0, 0.0, goal],
        obstacle_list=[], rand_area=[-2.0, 2.0], max_iter=0)
    planner.node_list = [planner.Node(0.0, 0.0, goal + 0.2),
                         planner.Node(0.0, 0.0, candidate)]

    assert planner.search_best_goal_node() == 1


@pytest.mark.parametrize("planner_class", [RRTDubins, RRTStarDubins])
@pytest.mark.parametrize(
    "candidate, goal",
    [(math.pi - 0.05, -math.pi + 0.05), (0.2, 0.0)],
)
def test_dubins_goal_rejects_wrong_heading(planner_class, candidate, goal):
    planner = planner_class(
        start=[-1.0, 0.0, 0.0], goal=[0.0, 0.0, goal],
        obstacle_list=[], rand_area=[-2.0, 2.0], max_iter=0)
    planner.node_list = [planner.Node(0.0, 0.0, candidate)]

    assert planner.search_best_goal_node() is None


@pytest.mark.parametrize(
    "candidate, goal",
    [(math.pi - 0.01, -math.pi + 0.01),
     (-math.pi + 0.01, math.pi - 0.01),
     (0.2 + 2 * math.pi, 0.2)],
)
def test_closed_loop_goal_accepts_equivalent_headings(candidate, goal):
    planner = closed_loop.ClosedLoopRRTStar(
        start=[-1.0, 0.0, 0.0], goal=[0.0, 0.0, goal],
        obstacle_list=[], rand_area=[-2.0, 2.0], max_iter=0)
    planner.node_list = [planner.Node(0.0, 0.0, goal + 0.2),
                         planner.Node(0.0, 0.0, candidate)]

    assert planner.get_goal_indexes() == [1]


@pytest.mark.parametrize(
    "candidate, goal",
    [(math.pi - 0.1, -math.pi + 0.1), (0.2, 0.0)],
)
def test_closed_loop_goal_rejects_wrong_heading(candidate, goal):
    planner = closed_loop.ClosedLoopRRTStar(
        start=[-1.0, 0.0, 0.0], goal=[0.0, 0.0, goal],
        obstacle_list=[], rand_area=[-2.0, 2.0], max_iter=0)
    planner.node_list = [planner.Node(0.0, 0.0, candidate)]

    assert planner.get_goal_indexes() == []


@pytest.mark.parametrize(
    "tracked, goal, expected",
    [(math.pi - 0.01, -math.pi + 0.01, True),
     (-math.pi + 0.01, math.pi - 0.01, True),
     (0.2, 0.2 + 2 * math.pi, True),
     (math.pi - 0.35, -math.pi + 0.35, False),
     (0.7, 0.0, False)],
)
def test_closed_loop_tracking_heading_wrap(monkeypatch, tracked, goal, expected):
    planner = closed_loop.ClosedLoopRRTStar(
        start=[0.0, 0.0, 0.0], goal=[1.0, 0.0, goal],
        obstacle_list=[], rand_area=[-2.0, 2.0], max_iter=0)
    monkeypatch.setattr(closed_loop.pure_pursuit, "extend_path",
                        lambda cx, cy, cyaw: (cx, cy, cyaw))
    monkeypatch.setattr(closed_loop.pure_pursuit, "calc_speed_profile",
                        lambda *args: [0.0, 0.0])

    def simulate_tracking(*args):
        return ([0.0, 1.0], [0.0, 1.0], [0.0, 0.0],
                [0.0, tracked], [0.0, 1.0], [0.0, 0.0],
                [0.0, 0.0], True)

    monkeypatch.setattr(closed_loop.pure_pursuit, "closed_loop_prediction",
                        simulate_tracking)
    monkeypatch.setattr(planner, "check_collision", lambda *args: True)

    path = [[1.0, 0.0, goal], [0.0, 0.0, 0.0]]
    feasible, *_ = planner.check_tracking_path_is_feasible(path)

    assert feasible is expected


if __name__ == "__main__":
    conftest.run_this_test(__file__)
