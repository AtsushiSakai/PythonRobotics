import conftest
from PathPlanning.GridBasedSweepCPP \
    import grid_based_sweep_coverage_path_planner

grid_based_sweep_coverage_path_planner.do_animation = False
RIGHT = grid_based_sweep_coverage_path_planner. \
    SweepSearcher.MovingDirection.RIGHT
LEFT = grid_based_sweep_coverage_path_planner. \
    SweepSearcher.MovingDirection.LEFT
UP = grid_based_sweep_coverage_path_planner. \
    SweepSearcher.SweepDirection.UP
DOWN = grid_based_sweep_coverage_path_planner. \
    SweepSearcher.SweepDirection.DOWN


def test_planning1():
    ox = [0.0, 20.0, 50.0, 100.0, 130.0, 40.0, 0.0]
    oy = [0.0, -20.0, 0.0, 30.0, 60.0, 80.0, 0.0]
    resolution = 5.0

    px, py = grid_based_sweep_coverage_path_planner.planning(
        ox, oy, resolution,
        moving_direction=RIGHT,
        sweeping_direction=DOWN,
    )
    assert len(px) >= 5

    px, py = grid_based_sweep_coverage_path_planner.planning(
        ox, oy, resolution,
        moving_direction=LEFT,
        sweeping_direction=DOWN,
    )
    assert len(px) >= 5

    px, py = grid_based_sweep_coverage_path_planner.planning(
        ox, oy, resolution,
        moving_direction=RIGHT,
        sweeping_direction=UP,
    )
    assert len(px) >= 5

    px, py = grid_based_sweep_coverage_path_planner.planning(
        ox, oy, resolution,
        moving_direction=RIGHT,
        sweeping_direction=UP,
    )
    assert len(px) >= 5


def test_planning2():
    ox = [0.0, 50.0, 50.0, 0.0, 0.0]
    oy = [0.0, 0.0, 30.0, 30.0, 0.0]
    resolution = 1.3

    px, py = grid_based_sweep_coverage_path_planner.planning(
        ox, oy, resolution,
        moving_direction=RIGHT,
        sweeping_direction=DOWN,
    )
    assert len(px) >= 5

    px, py = grid_based_sweep_coverage_path_planner.planning(
        ox, oy, resolution,
        moving_direction=LEFT,
        sweeping_direction=DOWN,
    )
    assert len(px) >= 5

    px, py = grid_based_sweep_coverage_path_planner.planning(
        ox, oy, resolution,
        moving_direction=RIGHT,
        sweeping_direction=UP,
    )
    assert len(px) >= 5

    px, py = grid_based_sweep_coverage_path_planner.planning(
        ox, oy, resolution,
        moving_direction=RIGHT,
        sweeping_direction=DOWN,
    )
    assert len(px) >= 5


def test_planning3():
    ox = [0.0, 20.0, 50.0, 200.0, 130.0, 40.0, 0.0]
    oy = [0.0, -80.0, 0.0, 30.0, 60.0, 80.0, 0.0]
    resolution = 5.1
    px, py = grid_based_sweep_coverage_path_planner.planning(
        ox, oy, resolution,
        moving_direction=RIGHT,
        sweeping_direction=DOWN,
    )
    assert len(px) >= 5

    px, py = grid_based_sweep_coverage_path_planner.planning(
        ox, oy, resolution,
        moving_direction=LEFT,
        sweeping_direction=DOWN,
    )
    assert len(px) >= 5

    px, py = grid_based_sweep_coverage_path_planner.planning(
        ox, oy, resolution,
        moving_direction=RIGHT,
        sweeping_direction=UP,
    )
    assert len(px) >= 5

    px, py = grid_based_sweep_coverage_path_planner.planning(
        ox, oy, resolution,
        moving_direction=RIGHT,
        sweeping_direction=DOWN,
    )
    assert len(px) >= 5


if __name__ == '__main__':
    conftest.run_this_test(__file__)
