import conftest
from PathPlanning.FrenetOptimalTrajectory import frenet_optimal_trajectory as m
from PathPlanning.FrenetOptimalTrajectory.frenet_optimal_trajectory import (
    LateralMovement,
    LongitudinalMovement,
)


def test1():
    m.show_animation = False
    m.SIM_LOOP = 5
    m.main()


def test2():
    m.show_animation = False
    m.LATERAL_MOVEMENT = LateralMovement.HIGH_SPEED
    m.LONGITUDINAL_MOVEMENT = LongitudinalMovement.MERGING_AND_STOPPING
    m.SIM_LOOP = 5
    m.main()


def test3():
    m.show_animation = False
    m.LATERAL_MOVEMENT = LateralMovement.HIGH_SPEED
    m.LONGITUDINAL_MOVEMENT = LongitudinalMovement.VELOCITY_KEEPING
    m.SIM_LOOP = 5
    m.main()


def test4():
    m.show_animation = False
    m.LATERAL_MOVEMENT = LateralMovement.HIGH_SPEED
    m.LONGITUDINAL_MOVEMENT = LongitudinalMovement.VELOCITY_KEEPING
    m.SIM_LOOP = 5
    m.main()


def test5():
    m.show_animation = False
    m.LATERAL_MOVEMENT = LateralMovement.HIGH_SPEED
    m.LONGITUDINAL_MOVEMENT = LongitudinalMovement.MERGING_AND_STOPPING
    m.SIM_LOOP = 5
    m.main()


if __name__ == "__main__":
    conftest.run_this_test(__file__)
