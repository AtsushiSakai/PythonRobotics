"""
Check that examples can be started from their own directory, as described in
"How to run sample codes" (``cd <example dir>`` and ``python <script>.py``).

Each script is loaded in a fresh interpreter whose working directory is the
script's directory, without the repository root on ``sys.path``.
``runpy.run_path`` does not set ``__name__`` to ``"__main__"``, so only the
module level code (the imports) runs, not the simulation.
"""
import os
import subprocess
import sys

import pytest

import conftest

SCRIPTS = [
    "ArmNavigation/two_joint_arm_to_point_control/"
    "two_joint_arm_to_point_control.py",
    "Mapping/ndt_map/ndt_map.py",
    "Mapping/normal_vector_estimation/normal_vector_estimation.py",
    "PathPlanning/ClosedLoopRRTStar/closed_loop_rrt_star_car.py",
    "PathPlanning/ClosedLoopRRTStar/pure_pursuit.py",
    "PathPlanning/ClosedLoopRRTStar/unicycle_model.py",
    "PathPlanning/ModelPredictiveTrajectoryGenerator/trajectory_generator.py",
    "PathPlanning/ModelPredictiveTrajectoryGenerator/lookup_table_generator.py",
    "PathPlanning/TimeBasedPathPlanning/GridWithDynamicObstacles.py",
    "PathPlanning/TimeBasedPathPlanning/PriorityBasedPlanner.py",
    "PathPlanning/TimeBasedPathPlanning/SafeInterval.py",
    "PathPlanning/TimeBasedPathPlanning/SpaceTimeAStar.py",
    "SLAM/EKFSLAM/ekf_slam.py",
    "SLAM/FastSLAM1/fast_slam1.py",
    "SLAM/FastSLAM2/fast_slam2.py",
]


@pytest.mark.parametrize("script", SCRIPTS)
def test_load_from_script_directory(script):
    script_path = os.path.join(conftest.ROOT_DIR, *script.split("/"))
    env = {key: value for key, value in os.environ.items()
           if key != "PYTHONPATH"}
    env["MPLBACKEND"] = "Agg"

    result = subprocess.run(
        [sys.executable, "-c",
         f"import runpy; runpy.run_path({script_path!r})"],
        cwd=os.path.dirname(script_path), env=env,
        capture_output=True, text=True)

    assert result.returncode == 0, result.stderr


if __name__ == '__main__':
    conftest.run_this_test(__file__)
