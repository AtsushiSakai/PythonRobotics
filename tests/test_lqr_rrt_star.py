import conftest  # Add root path to sys.path
from PathPlanning.LQRRRTStar import lqr_rrt_star as m
import random

random.seed(12345)


def test1():
    m.show_animation = False
    m.main(maxIter=5)


def test_steer_handles_failed_local_planner():
    planner = m.LQRRRTStar(
        start=[0.0, 0.0],
        goal=[1.0, 1.0],
        obstacle_list=[],
        rand_area=[-1.0, 1.0],
    )
    planner.lqr_planner.MAX_TIME = 0.0

    result = planner.steer(planner.start, planner.end)

    assert result is None


if __name__ == '__main__':
    conftest.run_this_test(__file__)
