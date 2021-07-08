import conftest
import numpy as np
from ArmNavigation.dynamic_movement_primitives import dynamic_movement_primitives

def test_1():
    # test that functions work at all
    DMP_controller = dynamic_movement_primitives.DMP()
    DMP_controller.solve_trajectory(0,3,3, visualize=False)

def test_2():
    # test that trajectory can be learned from user-passed data
    t = np.arange(0,5,0.01)
    sin_t = np.sin(t)

    train_data = [t,sin_t]
    DMP_controller = dynamic_movement_primitives.DMP(training_data=train_data)
    DMP_controller.solve_trajectory(-3,3,4, visualize=False)

def test_3():
    # check that learned trajectory is close to initial
    t = np.arange(0,3*np.pi/2,0.01)
    sin_t = np.sin(t)

    train_data = [t,sin_t]
    DMP_controller = dynamic_movement_primitives.DMP(dt=0.01, timesteps=len(t), training_data=train_data)
    t, pos = DMP_controller.solve_trajectory(0,-1,3*np.pi/2, visualize=False)

    diff = abs(pos - sin_t)
    assert(max(diff) < 0.1)

if __name__ == '__main__':
    conftest.run_this_test(__file__)
