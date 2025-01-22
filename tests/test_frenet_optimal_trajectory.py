import conftest
from PathPlanning.FrenetOptimalTrajectory import frenet_optimal_trajectory as m
from PathPlanning.FrenetOptimalTrajectory.frenet_optimal_trajectory import (
    LateralMovement,
    LongitudinalMovement,
)


def default_scenario_test():
    m.show_animation = False
    m.SIM_LOOP = 5
    m.main()


def high_speed_and_merging_and_stopping_scenario_test():
    m.show_animation = False
    m.LATERAL_MOVEMENT = LateralMovement.HIGH_SPEED
    m.LONGITUDINAL_MOVEMENT = LongitudinalMovement.MERGING_AND_STOPPING
    m.SIM_LOOP = 5
    m.main()


def high_speed_and_velocity_keeping_scenario_test():
    m.show_animation = False
    m.LATERAL_MOVEMENT = LateralMovement.HIGH_SPEED
    m.LONGITUDINAL_MOVEMENT = LongitudinalMovement.VELOCITY_KEEPING
    m.SIM_LOOP = 5
    m.main()


def low_speed_and_velocity_keeping_scenario_test():
    m.show_animation = False
    m.LATERAL_MOVEMENT = LateralMovement.LOW_SPEED
    m.LONGITUDINAL_MOVEMENT = LongitudinalMovement.VELOCITY_KEEPING
    m.SIM_LOOP = 5
    m.main()


def low_speed_and_merging_and_stopping_scenario_test():
    m.show_animation = False
    m.LATERAL_MOVEMENT = LateralMovement.LOW_SPEED
    m.LONGITUDINAL_MOVEMENT = LongitudinalMovement.MERGING_AND_STOPPING
    m.SIM_LOOP = 5
    m.main()


if __name__ == "__main__":
    conftest.run_this_test(__file__)
