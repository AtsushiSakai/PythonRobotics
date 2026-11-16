import conftest  # Add root path to sys.path
import numpy as np
from scipy.linalg import solve_discrete_are
from PathPlanning.LQRPlanner import lqr_planner as m


def test_1():
    m.SHOW_ANIMATION = False
    m.main()


def test_dare_uses_matrix_products():
    planner = m.LQRPlanner()
    planner.EPS = 1e-10
    A, B = planner.get_system_model()
    Q, R = np.eye(2), np.eye(1)

    expected = solve_discrete_are(A, B, Q, R)
    np.testing.assert_allclose(
        planner.solve_dare(A, B, Q, R), expected, rtol=1e-8, atol=1e-8
    )

    gain, _, _ = planner.dlqr(A, B, Q, R)
    expected_gain = np.linalg.solve(R + B.T @ expected @ B, B.T @ expected @ A)
    np.testing.assert_allclose(gain, expected_gain, rtol=1e-8, atol=1e-8)


def test_dare_with_multiple_control_inputs():
    planner = m.LQRPlanner()
    planner.EPS = 1e-10
    A = np.array([[0.9, 0.2], [-0.1, 0.8]])
    B = np.eye(2)
    Q = np.diag([2.0, 1.0])
    R = np.diag([0.3, 0.7])

    expected = solve_discrete_are(A, B, Q, R)
    np.testing.assert_allclose(
        planner.solve_dare(A, B, Q, R), expected, rtol=1e-8, atol=1e-8
    )


if __name__ == '__main__':
    conftest.run_this_test(__file__)
