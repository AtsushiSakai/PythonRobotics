import conftest
import numpy as np
from PathPlanning.DynamicMovementPrimitives import \
            dynamic_movement_primitives


def test_1():
    # test that trajectory can be learned from user-passed data
    T = 5
    t = np.arange(0, T, 0.01)
    sin_t = np.sin(t)
    train_data = np.array([t, sin_t]).T

    DMP_controller = dynamic_movement_primitives.DMP(train_data, T)
    DMP_controller.recreate_trajectory(train_data[0], train_data[-1], 4)


def test_2():
    # test that length of trajectory is equal to desired number of timesteps
    T = 5
    t = np.arange(0, T, 0.01)
    sin_t = np.sin(t)
    train_data = np.array([t, sin_t]).T

    DMP_controller = dynamic_movement_primitives.DMP(train_data, T)
    t, path = DMP_controller.recreate_trajectory(train_data[0],
                                                 train_data[-1], 4)

    assert(path.shape[0] == DMP_controller.timesteps)


def test_3():
    # check that learned trajectory is close to initial
    T = 3*np.pi/2
    A_noise = 0.02
    t = np.arange(0, T, 0.01)
    noisy_sin_t = np.sin(t) + A_noise*np.random.rand(len(t))
    train_data = np.array([t, noisy_sin_t]).T

    DMP_controller = dynamic_movement_primitives.DMP(train_data, T)
    t, pos = DMP_controller.recreate_trajectory(train_data[0],
                                                train_data[-1], T)

    diff = abs(pos[:, 1] - noisy_sin_t)
    assert(max(diff) < 5*A_noise)


if __name__ == '__main__':
    conftest.run_this_test(__file__)
