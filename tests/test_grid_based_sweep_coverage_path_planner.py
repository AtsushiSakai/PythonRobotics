import os
import sys
from unittest import TestCase

sys.path.append(os.path.dirname(
    os.path.abspath(__file__)) + "/../PathPlanning/GridBasedSweepCPP")
sys.path.append(
    os.path.dirname(os.path.abspath(__file__)) + "/../Mapping/grid_map_lib")
try:
    import grid_based_sweep_coverage_path_planner
except ImportError:
    raise

grid_based_sweep_coverage_path_planner.do_animation = False


class TestPlanning(TestCase):

    RIGHT = grid_based_sweep_coverage_path_planner.\
        SweepSearcher.MovingDirection.RIGHT
    LEFT = grid_based_sweep_coverage_path_planner. \
        SweepSearcher.MovingDirection.LEFT
    UP = grid_based_sweep_coverage_path_planner. \
        SweepSearcher.SweepDirection.UP
    DOWN = grid_based_sweep_coverage_path_planner. \
        SweepSearcher.SweepDirection.DOWN

    def test_planning1(self):
        ox = [0.0, 20.0, 50.0, 100.0, 130.0, 40.0, 0.0]
        oy = [0.0, -20.0, 0.0, 30.0, 60.0, 80.0, 0.0]
        resolution = 5.0

        px, py = grid_based_sweep_coverage_path_planner.planning(
            ox, oy, resolution,
            moving_direction=self.RIGHT,
            sweeping_direction=self.DOWN,
        )
        self.assertGreater(len(px), 5)

        px, py = grid_based_sweep_coverage_path_planner.planning(
            ox, oy, resolution,
            moving_direction=self.LEFT,
            sweeping_direction=self.DOWN,
        )
        self.assertGreater(len(px), 5)

        px, py = grid_based_sweep_coverage_path_planner.planning(
            ox, oy, resolution,
            moving_direction=self.RIGHT,
            sweeping_direction=self.UP,
        )
        self.assertGreater(len(px), 5)

        px, py = grid_based_sweep_coverage_path_planner.planning(
            ox, oy, resolution,
            moving_direction=self.RIGHT,
            sweeping_direction=self.UP,
        )
        self.assertGreater(len(px), 5)

    def test_planning2(self):
        ox = [0.0, 50.0, 50.0, 0.0, 0.0]
        oy = [0.0, 0.0, 30.0, 30.0, 0.0]
        resolution = 1.3

        px, py = grid_based_sweep_coverage_path_planner.planning(
            ox, oy, resolution,
            moving_direction=self.RIGHT,
            sweeping_direction=self.DOWN,
        )
        self.assertGreater(len(px), 5)

        px, py = grid_based_sweep_coverage_path_planner.planning(
            ox, oy, resolution,
            moving_direction=self.LEFT,
            sweeping_direction=self.DOWN,
        )
        self.assertGreater(len(px), 5)

        px, py = grid_based_sweep_coverage_path_planner.planning(
            ox, oy, resolution,
            moving_direction=self.RIGHT,
            sweeping_direction=self.UP,
        )
        self.assertGreater(len(px), 5)

        px, py = grid_based_sweep_coverage_path_planner.planning(
            ox, oy, resolution,
            moving_direction=self.RIGHT,
            sweeping_direction=self.DOWN,
        )
        self.assertGreater(len(px), 5)

    def test_planning3(self):
        ox = [0.0, 20.0, 50.0, 200.0, 130.0, 40.0, 0.0]
        oy = [0.0, -80.0, 0.0, 30.0, 60.0, 80.0, 0.0]
        resolution = 5.1
        px, py = grid_based_sweep_coverage_path_planner.planning(
            ox, oy, resolution,
            moving_direction=self.RIGHT,
            sweeping_direction=self.DOWN,
        )
        self.assertGreater(len(px), 5)

        px, py = grid_based_sweep_coverage_path_planner.planning(
            ox, oy, resolution,
            moving_direction=self.LEFT,
            sweeping_direction=self.DOWN,
        )
        self.assertGreater(len(px), 5)

        px, py = grid_based_sweep_coverage_path_planner.planning(
            ox, oy, resolution,
            moving_direction=self.RIGHT,
            sweeping_direction=self.UP,
        )
        self.assertGreater(len(px), 5)

        px, py = grid_based_sweep_coverage_path_planner.planning(
            ox, oy, resolution,
            moving_direction=self.RIGHT,
            sweeping_direction=self.DOWN,
        )
        self.assertGreater(len(px), 5)
