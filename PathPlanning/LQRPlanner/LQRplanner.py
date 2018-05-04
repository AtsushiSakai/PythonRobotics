"""

LQR local path planning module

author: Atsushi Sakai (@Atsushi_twi)

"""

import matplotlib.pyplot as plt
import numpy as np
import scipy.linalg as la
import math

show_animation = True

MAX_TIME = 100.0
DT = 0.1


def LQRplanning(sx, sy, gx, gy):

    rx, ry = [sx], [sy]

    x = np.matrix([gx - sx, gy - sy]).T  # State vector

    # Linear system model
    A, B = get_system_model()

    time = 0.0

    while time <= MAX_TIME:
        time += DT

        u = LQR_control(A, B, x)

        x = A * x + B * u

        rx.append(x[0, 0])
        ry.append(x[1, 0])

        plt.plot(rx, ry)
        plt.plot(rx[-1], ry[-1], "xr")
        plt.pause(1.0)

        d = math.sqrt((gx - x[0, 0])**2 + (gy - x[1, 0])**2)
        print(d)
        if d <= 0.1:
            print("Goal!!")
            break

    return rx, ry


def solve_DARE(A, B, Q, R):
    """
    solve a discrete time_Algebraic Riccati equation (DARE)
    """
    X = Q
    maxiter = 150
    eps = 0.01

    for i in range(maxiter):
        Xn = A.T * X * A - A.T * X * B * \
            la.inv(R + B.T * X * B) * B.T * X * A + Q
        if (abs(Xn - X)).max() < eps:
            X = Xn
            break
        X = Xn

    return Xn


def dlqr(A, B, Q, R):
    """Solve the discrete time lqr controller.
    x[k+1] = A x[k] + B u[k]
    cost = sum x[k].T*Q*x[k] + u[k].T*R*u[k]
    # ref Bertsekas, p.151
    """

    # first, try to solve the ricatti equation
    X = solve_DARE(A, B, Q, R)

    # compute the LQR gain
    K = np.matrix(la.inv(B.T * X * B + R) * (B.T * X * A))

    eigVals, eigVecs = la.eig(A - B * K)

    return K, X, eigVals


def get_system_model():
    A = np.eye(2) * DT
    A[0, 1] = 1.0
    B = np.matrix([0.0, 1.0]).T

    return A, B


def LQR_control(A, B, x):

    Kopt, X, ev = dlqr(A, B, np.eye(2), np.eye(1))

    u = -Kopt * x

    return u


def main():
    print(__file__ + " start!!")

    sx = -10.0
    sy = -5.0
    gx = 0.0
    gy = 0.0

    rx, ry = LQRplanning(sx, sy, gx, gy)

    plt.plot(sx, sy, "xb")
    plt.plot(gx, gy, "xb")
    plt.plot(rx, ry)
    plt.axis("equal")
    plt.show()


if __name__ == '__main__':
    main()
