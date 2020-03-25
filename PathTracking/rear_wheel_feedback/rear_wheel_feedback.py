"""

Path tracking simulation with rear wheel feedback steering control and PID speed control.

author: Atsushi Sakai(@Atsushi_twi)

"""
import matplotlib.pyplot as plt
import math
import numpy as np

from scipy import interpolate
from scipy import optimize

Kp = 1.0  # speed propotional gain
# steering control parameter
KTH = 1.0
KE = 0.5

dt = 0.1  # [s]
L = 2.9  # [m]

show_animation = True

class State:
    def __init__(self, x=0.0, y=0.0, yaw=0.0, v=0.0, direction=1):
        self.x = x
        self.y = y
        self.yaw = yaw
        self.v = v
        self.direction = direction

    def update(self, a, delta, dt):
        self.x   = self.x + self.v * math.cos(self.yaw) * dt
        self.y   = self.y + self.v * math.sin(self.yaw) * dt
        self.yaw = self.yaw + self.v / L * math.tan(delta) * dt
        self.v   = self.v + a * dt

class CubicSplinePath:
    def __init__(self, x, y):
        x, y = map(np.asarray, (x, y))
        s = np.append([0],(np.cumsum(np.diff(x)**2) + np.cumsum(np.diff(y)**2))**0.5)

        self.X = interpolate.CubicSpline(s, x)
        self.Y = interpolate.CubicSpline(s, y)

        self.dX = self.X.derivative(1)
        self.ddX = self.X.derivative(2)

        self.dY = self.Y.derivative(1)
        self.ddY = self.Y.derivative(2)

        self.length = s[-1]
    
    def calc_yaw(self, s):
        dx, dy = self.dX(s), self.dY(s)
        return np.arctan2(dy, dx)
    
    def calc_curvature(self, s):
        dx, dy   = self.dX(s), self.dY(s)
        ddx, ddy   = self.ddX(s), self.ddY(s)
        return (ddy * dx - ddx * dy) / ((dx ** 2 + dy ** 2)**(3 / 2))
    
    def __find_nearest_point(self, s0, x, y):
        def calc_distance(_s, *args):
            _x, _y= self.X(_s), self.Y(_s)
            return (_x - args[0])**2 + (_y - args[1])**2
        
        def calc_distance_jacobian(_s, *args):
            _x, _y = self.X(_s), self.Y(_s)
            _dx, _dy = self.dX(_s), self.dY(_s)
            return 2*_dx*(_x - args[0])+2*_dy*(_y-args[1])

        minimum = optimize.fmin_cg(calc_distance, s0, calc_distance_jacobian, args=(x, y), full_output=True, disp=False)
        return minimum

    def calc_track_error(self, x, y, s0):
        ret = self.__find_nearest_point(s0, x, y)
        
        s = ret[0][0]
        e = ret[1]

        k   = self.calc_curvature(s)
        yaw = self.calc_yaw(s)

        dxl = self.X(s) - x
        dyl = self.Y(s) - y
        angle = pi_2_pi(yaw - math.atan2(dyl, dxl))
        if angle < 0:
            e*= -1

        return e, k, yaw, s

def pid_control(target, current):
    a = Kp * (target - current)
    return a

def pi_2_pi(angle):
    while(angle > math.pi):
        angle = angle - 2.0 * math.pi

    while(angle < -math.pi):
        angle = angle + 2.0 * math.pi

    return angle

def rear_wheel_feedback_control(state, e, k, yaw_ref):
    v = state.v
    th_e = pi_2_pi(state.yaw - yaw_ref)

    omega = v * k * math.cos(th_e) / (1.0 - k * e) - \
        KTH * abs(v) * th_e - KE * v * math.sin(th_e) * e / th_e

    if th_e == 0.0 or omega == 0.0:
        return 0.0

    delta = math.atan2(L * omega / v, 1.0)

    return delta


def simulate(path_ref, goal):
    T = 500.0  # max simulation time
    goal_dis = 0.3

    state = State(x=-0.0, y=-0.0, yaw=0.0, v=0.0)

    time = 0.0
    x = [state.x]
    y = [state.y]
    yaw = [state.yaw]
    v = [state.v]
    t = [0.0]
    goal_flag = False

    s = np.arange(0, path_ref.length, 0.1)
    e, k, yaw_ref, s0 = path_ref.calc_track_error(state.x, state.y, 0.0)

    while T >= time:
        e, k, yaw_ref, s0 = path_ref.calc_track_error(state.x, state.y, s0)
        di = rear_wheel_feedback_control(state, e, k, yaw_ref)

        speed_ref = calc_target_speed(state, yaw_ref)
        ai = pid_control(speed_ref, state.v)
        state.update(ai, di, dt)

        time = time + dt

        # check goal
        dx = state.x - goal[0]
        dy = state.y - goal[1]
        if math.hypot(dx, dy) <= goal_dis:
            print("Goal")
            goal_flag = True
            break

        x.append(state.x)
        y.append(state.y)
        yaw.append(state.yaw)
        v.append(state.v)
        t.append(time)

        if show_animation:
            plt.cla()
            # for stopping simulation with the esc key.
            plt.gcf().canvas.mpl_connect('key_release_event',
                    lambda event: [exit(0) if event.key == 'escape' else None])
            plt.plot(path_ref.X(s), path_ref.Y(s), "-r", label="course")
            plt.plot(x, y, "ob", label="trajectory")
            plt.plot(path_ref.X(s0), path_ref.Y(s0), "xg", label="target")
            plt.axis("equal")
            plt.grid(True)
            plt.title("speed[km/h]:{:.2f}, target s-param:{:.2f}".format(round(state.v * 3.6, 2), s0))
            plt.pause(0.0001)

    return t, x, y, yaw, v, goal_flag

def calc_target_speed(state, yaw_ref):
    target_speed = 10.0 / 3.6

    dyaw = yaw_ref - state.yaw
    switch = math.pi / 4.0 <= dyaw < math.pi / 2.0

    if switch:
        state.direction *= -1
        return 0.0
    
    if state.direction != 1:
        return -target_speed

    return target_speed

def main():
    print("rear wheel feedback tracking start!!")
    ax = [0.0, 6.0, 12.5, 5.0, 7.5, 3.0, -1.0]
    ay = [0.0, 0.0, 5.0, 6.5, 3.0, 5.0, -2.0]
    goal = [ax[-1], ay[-1]]

    reference_path = CubicSplinePath(ax, ay)
    s = np.arange(0, reference_path.length, 0.1)

    t, x, y, yaw, v, goal_flag = simulate(reference_path, goal)

    # Test
    assert goal_flag, "Cannot goal"

    if show_animation:  # pragma: no cover
        plt.close()
        plt.subplots(1)
        plt.plot(ax, ay, "xb", label="input")
        plt.plot(reference_path.X(s), reference_path.Y(s), "-r", label="spline")
        plt.plot(x, y, "-g", label="tracking")
        plt.grid(True)
        plt.axis("equal")
        plt.xlabel("x[m]")
        plt.ylabel("y[m]")
        plt.legend()

        plt.subplots(1)
        plt.plot(s, np.rad2deg(reference_path.calc_yaw(s)), "-r", label="yaw")
        plt.grid(True)
        plt.legend()
        plt.xlabel("line length[m]")
        plt.ylabel("yaw angle[deg]")

        plt.subplots(1)
        plt.plot(s, reference_path.calc_curvature(s), "-r", label="curvature")
        plt.grid(True)
        plt.legend()
        plt.xlabel("line length[m]")
        plt.ylabel("curvature [1/m]")

        plt.show()

if __name__ == '__main__':
    main()
