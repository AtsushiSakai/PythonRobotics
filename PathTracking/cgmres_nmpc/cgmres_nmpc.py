"""

Nonlinear MPC simulation with CGMRES

author Atsushi Sakai (@Atsushi_twi)

Ref:

- Shunichi09/nonlinear_control: Implementing the nonlinear model predictive control, sliding mode control https://github.com/Shunichi09/nonlinear_control

"""

import numpy as np
import matplotlib.pyplot as plt
import math

U_A_MAX = 1.0
U_OMEGA_MAX = math.radians(45.0)
PHI_V = 0.01
PHI_OMEGA = 0.01
WB = 0.25  # [m] wheel base

show_animation = True


def differential_model(v, yaw, u_1, u_2):

    dx = math.cos(yaw) * v
    dy = math.sin(yaw) * v
    dv = u_1
    dyaw = v / WB * math.sin(u_2)  # tan is not good for nonlinear optimization

    return dx, dy, dyaw, dv


class TwoWheeledSystem():

    def __init__(self, init_x, init_y, init_yaw, init_v):

        self.x = init_x
        self.y = init_y
        self.yaw = init_yaw
        self.v = init_v
        self.history_x = [init_x]
        self.history_y = [init_y]
        self.history_yaw = [init_yaw]
        self.history_v = [init_v]

    def update_state(self, u_1, u_2, dt=0.01):

        dx, dy, dyaw, dv = differential_model(self.v, self.yaw, u_1, u_2)

        self.x += dt * dx
        self.y += dt * dy
        self.yaw += dt * dyaw
        self.v += dt * dv

        # save
        self.history_x.append(self.x)
        self.history_y.append(self.y)
        self.history_yaw.append(self.yaw)
        self.history_v.append(self.v)


class NMPCSimulatorSystem():

    def calc_predict_and_adjoint_state(self, x, y, yaw, v, u_1s, u_2s, N, dt):
        x_s, y_s, yaw_s, v_s = self._calc_predict_states(
            x, y, yaw, v, u_1s, u_2s, N, dt)  # by using state equation
        lam_1s, lam_2s, lam_3s, lam_4s = self._calc_adjoint_states(
            x_s, y_s, yaw_s, v_s, u_1s, u_2s, N, dt)  # by using adjoint equation

        return x_s, y_s, yaw_s, v_s, lam_1s, lam_2s, lam_3s, lam_4s

    def _calc_predict_states(self, x, y, yaw, v, u_1s, u_2s, N, dt):
        x_s = [x]
        y_s = [y]
        yaw_s = [yaw]
        v_s = [v]

        for i in range(N):
            temp_x_1, temp_x_2, temp_x_3, temp_x_4 = self._predict_state_with_oylar(
                x_s[i], y_s[i], yaw_s[i], v_s[i], u_1s[i], u_2s[i], dt)
            x_s.append(temp_x_1)
            y_s.append(temp_x_2)
            yaw_s.append(temp_x_3)
            v_s.append(temp_x_4)

        return x_s, y_s, yaw_s, v_s

    def _calc_adjoint_states(self, x_s, y_s, yaw_s, v_s, u_1s, u_2s, N, dt):
        lam_1s = [x_s[-1]]
        lam_2s = [y_s[-1]]
        lam_3s = [yaw_s[-1]]
        lam_4s = [v_s[-1]]

        # backward adjoint state calc
        for i in range(N - 1, 0, -1):
            temp_lam_1, temp_lam_2, temp_lam_3, temp_lam_4 = self._adjoint_state_with_oylar(
                x_s[i], y_s[i], yaw_s[i], v_s[i], lam_1s[0], lam_2s[0], lam_3s[0], lam_4s[0],
                u_1s[i], u_2s[i], dt)
            lam_1s.insert(0, temp_lam_1)
            lam_2s.insert(0, temp_lam_2)
            lam_3s.insert(0, temp_lam_3)
            lam_4s.insert(0, temp_lam_4)

        return lam_1s, lam_2s, lam_3s, lam_4s

    def _predict_state_with_oylar(self, x, y, yaw, v, u_1, u_2, dt):

        dx, dy, dyaw, dv = differential_model(
            v, yaw, u_1, u_2)

        next_x_1 = x + dt * dx
        next_x_2 = y + dt * dy
        next_x_3 = yaw + dt * dyaw
        next_x_4 = v + dt * dv

        return next_x_1, next_x_2, next_x_3, next_x_4

    def _adjoint_state_with_oylar(self, x, y, yaw, v, lam_1, lam_2, lam_3, lam_4, u_1, u_2, dt):

        # ∂H/∂x
        pre_lam_1 = lam_1 + dt * 0.0
        pre_lam_2 = lam_2 + dt * 0.0
        pre_lam_3 = lam_3 + dt * \
            (- lam_1 * math.sin(yaw) * v + lam_2 * math.cos(yaw) * v)
        pre_lam_4 = lam_4 + dt * \
            (lam_1 * math.cos(yaw) + lam_2 *
             math.sin(yaw) + lam_3 * math.sin(u_2) / WB)

        return pre_lam_1, pre_lam_2, pre_lam_3, pre_lam_4


class NMPCController_with_CGMRES():
    """
    Attributes
    ------------
    zeta : float
        gain of optimal answer stability
    ht : float
        update value of NMPC this should be decided by zeta
    tf : float
        predict time
    alpha : float
        gain of predict time
    N : int
        predicte step, discritize value
    threshold : float
        cgmres's threshold value
    input_num : int
        system input length, this should include dummy u and constraint variables
    max_iteration : int
        decide by the solved matrix size
    simulator : NMPCSimulatorSystem class
    u_1s : list of float
        estimated optimal system input
    u_2s : list of float
        estimated optimal system input
    dummy_u_1s : list of float
        estimated dummy input
    dummy_u_2s : list of float
        estimated dummy input
    raw_1s : list of float
        estimated constraint variable
    raw_2s : list of float
        estimated constraint variable
    history_u_1 : list of float
        time history of actual system input
    history_u_2 : list of float
        time history of actual system input
    history_dummy_u_1 : list of float
        time history of actual dummy u_1
    history_dummy_u_2 : list of float
        time history of actual dummy u_2
    history_raw_1 : list of float
        time history of actual raw_1
    history_raw_2 : list of float
        time history of actual raw_2
    history_f : list of float
        time history of error of optimal
    """

    def __init__(self):
        # parameters
        self.zeta = 100.  # stability gain
        self.ht = 0.01  # difference approximation tick
        self.tf = 3.0  # final time
        self.alpha = 0.5  # time gain
        self.N = 10  # division number
        self.threshold = 0.001
        self.input_num = 6  # input number of dummy, constraints
        self.max_iteration = self.input_num * self.N

        # simulator
        self.simulator = NMPCSimulatorSystem()

        # initial input, initialize as 1.0
        self.u_1s = np.ones(self.N)
        self.u_2s = np.ones(self.N)
        self.dummy_u_1s = np.ones(self.N)
        self.dummy_u_2s = np.ones(self.N)
        self.raw_1s = np.zeros(self.N)
        self.raw_2s = np.zeros(self.N)

        self.history_u_1 = []
        self.history_u_2 = []
        self.history_dummy_u_1 = []
        self.history_dummy_u_2 = []
        self.history_raw_1 = []
        self.history_raw_2 = []
        self.history_f = []

    def calc_input(self, x, y, yaw, v, time):

        # calculating sampling time
        dt = self.tf * (1. - np.exp(-self.alpha * time)) / float(self.N)

        # x_dot
        x_1_dot, x_2_dot, x_3_dot, x_4_dot = differential_model(
            v, yaw, self.u_1s[0], self.u_2s[0])

        dx_1 = x_1_dot * self.ht
        dx_2 = x_2_dot * self.ht
        dx_3 = x_3_dot * self.ht
        dx_4 = x_4_dot * self.ht

        x_s, y_s, yaw_s, v_s, lam_1s, lam_2s, lam_3s, lam_4s = self.simulator.calc_predict_and_adjoint_state(
            x + dx_1, y + dx_2, yaw + dx_3, v + dx_4, self.u_1s, self.u_2s, self.N, dt)

        # Fxt:F(U,x+hx˙,t+h)
        Fxt = self._calc_f(x_s, y_s, yaw_s, v_s, lam_1s, lam_2s, lam_3s, lam_4s,
                           self.u_1s, self.u_2s, self.dummy_u_1s, self.dummy_u_2s,
                           self.raw_1s, self.raw_2s, self.N, dt)

        # F:F(U,x,t)
        x_s, y_s, yaw_s, v_s, lam_1s, lam_2s, lam_3s, lam_4s = self.simulator.calc_predict_and_adjoint_state(
            x, y, yaw, v, self.u_1s, self.u_2s, self.N, dt)

        F = self._calc_f(x_s, y_s, yaw_s, v_s, lam_1s, lam_2s, lam_3s, lam_4s,
                         self.u_1s, self.u_2s, self.dummy_u_1s, self.dummy_u_2s,
                         self.raw_1s, self.raw_2s, self.N, dt)

        right = -self.zeta * F - ((Fxt - F) / self.ht)

        du_1 = self.u_1s * self.ht
        du_2 = self.u_2s * self.ht
        ddummy_u_1 = self.dummy_u_1s * self.ht
        ddummy_u_2 = self.dummy_u_2s * self.ht
        draw_1 = self.raw_1s * self.ht
        draw_2 = self.raw_2s * self.ht

        x_s, y_s, yaw_s, v_s, lam_1s, lam_2s, lam_3s, lam_4s = self.simulator.calc_predict_and_adjoint_state(
            x + dx_1, y + dx_2, yaw + dx_3, v + dx_4, self.u_1s + du_1, self.u_2s + du_2, self.N, dt)

        # Fuxt:F(U+hdU(0),x+hx˙,t+h)
        Fuxt = self._calc_f(x_s, y_s, yaw_s, v_s, lam_1s, lam_2s, lam_3s, lam_4s,
                            self.u_1s + du_1, self.u_2s + du_2,
                            self.dummy_u_1s + ddummy_u_1, self.dummy_u_2s + ddummy_u_2,
                            self.raw_1s + draw_1, self.raw_2s + draw_2, self.N, dt)

        left = ((Fuxt - Fxt) / self.ht)

        # calculationg cgmres
        r0 = right - left
        r0_norm = np.linalg.norm(r0)

        vs = np.zeros((self.max_iteration, self.max_iteration + 1))
        vs[:, 0] = r0 / r0_norm

        hs = np.zeros((self.max_iteration + 1, self.max_iteration + 1))

        # in this case the state is 3(u and dummy_u)
        e = np.zeros((self.max_iteration + 1, 1))
        e[0] = 1.0

        ys_pre = None

        for i in range(self.max_iteration):
            du_1 = vs[::self.input_num, i] * self.ht
            du_2 = vs[1::self.input_num, i] * self.ht
            ddummy_u_1 = vs[2::self.input_num, i] * self.ht
            ddummy_u_2 = vs[3::self.input_num, i] * self.ht
            draw_1 = vs[4::self.input_num, i] * self.ht
            draw_2 = vs[5::self.input_num, i] * self.ht

            x_s, y_s, yaw_s, v_s, lam_1s, lam_2s, lam_3s, lam_4s = self.simulator.calc_predict_and_adjoint_state(
                x + dx_1, y + dx_2, yaw + dx_3, v + dx_4, self.u_1s + du_1, self.u_2s + du_2, self.N, dt)

            Fuxt = self._calc_f(x_s, y_s, yaw_s, v_s, lam_1s, lam_2s, lam_3s, lam_4s,
                                self.u_1s + du_1, self.u_2s + du_2,
                                self.dummy_u_1s + ddummy_u_1, self.dummy_u_2s + ddummy_u_2,
                                self.raw_1s + draw_1, self.raw_2s + draw_2, self.N, dt)

            Av = ((Fuxt - Fxt) / self.ht)

            sum_Av = np.zeros(self.max_iteration)

            # Gram–Schmidt orthonormalization
            for j in range(i + 1):
                hs[j, i] = np.dot(Av, vs[:, j])
                sum_Av = sum_Av + hs[j, i] * vs[:, j]

            v_est = Av - sum_Av

            hs[i + 1, i] = np.linalg.norm(v_est)

            vs[:, i + 1] = v_est / hs[i + 1, i]

            inv_hs = np.linalg.pinv(hs[:i + 1, :i])
            ys = np.dot(inv_hs, r0_norm * e[:i + 1])

            judge_value = r0_norm * e[:i + 1] - np.dot(hs[:i + 1, :i], ys[:i])

            if np.linalg.norm(judge_value) < self.threshold or i == self.max_iteration - 1:
                update_value = np.dot(vs[:, :i - 1], ys_pre[:i - 1]).flatten()
                du_1_new = du_1 + update_value[::self.input_num]
                du_2_new = du_2 + update_value[1::self.input_num]
                ddummy_u_1_new = ddummy_u_1 + update_value[2::self.input_num]
                ddummy_u_2_new = ddummy_u_2 + update_value[3::self.input_num]
                draw_1_new = draw_1 + update_value[4::self.input_num]
                draw_2_new = draw_2 + update_value[5::self.input_num]
                break

            ys_pre = ys

        # update input
        self.u_1s += du_1_new * self.ht
        self.u_2s += du_2_new * self.ht
        self.dummy_u_1s += ddummy_u_1_new * self.ht
        self.dummy_u_2s += ddummy_u_2_new * self.ht
        self.raw_1s += draw_1_new * self.ht
        self.raw_2s += draw_2_new * self.ht

        x_s, y_s, yaw_s, v_s, lam_1s, lam_2s, lam_3s, lam_4s = self.simulator.calc_predict_and_adjoint_state(
            x, y, yaw, v, self.u_1s, self.u_2s, self.N, dt)

        F = self._calc_f(x_s, y_s, yaw_s, v_s, lam_1s, lam_2s, lam_3s, lam_4s,
                         self.u_1s, self.u_2s, self.dummy_u_1s, self.dummy_u_2s,
                         self.raw_1s, self.raw_2s, self.N, dt)

        print("norm(F) = {0}".format(np.linalg.norm(F)))

        # for save
        self.history_f.append(np.linalg.norm(F))
        self.history_u_1.append(self.u_1s[0])
        self.history_u_2.append(self.u_2s[0])
        self.history_dummy_u_1.append(self.dummy_u_1s[0])
        self.history_dummy_u_2.append(self.dummy_u_2s[0])
        self.history_raw_1.append(self.raw_1s[0])
        self.history_raw_2.append(self.raw_2s[0])

        return self.u_1s, self.u_2s

    def _calc_f(self, x_s, y_s, yaw_s, v_s, lam_1s, lam_2s, lam_3s, lam_4s,
                u_1s, u_2s, dummy_u_1s, dummy_u_2s, raw_1s, raw_2s, N, dt):

        F = []
        for i in range(N):
            # ∂H/∂u(xi, ui, λi)
            F.append(u_1s[i] + lam_4s[i] + 2.0 * raw_1s[i] * u_1s[i])
            F.append(u_2s[i] + lam_3s[i] * v_s[i] /
                     WB * math.cos(u_2s[i])**2 + 2.0 * raw_2s[i] * u_2s[i])
            F.append(-PHI_V + 2.0 * raw_1s[i] * dummy_u_1s[i])
            F.append(-PHI_OMEGA + 2.0 * raw_2s[i] * dummy_u_2s[i])

            # C(xi, ui, λi)
            F.append(u_1s[i]**2 + dummy_u_1s[i]**2 - U_A_MAX**2)
            F.append(u_2s[i]**2 + dummy_u_2s[i]**2 - U_OMEGA_MAX**2)

        return np.array(F)


def plot_figures(plant_system, controller, iteration_num, dt):  # pragma: no cover
    # figure
    # time history
    fig_p = plt.figure()
    fig_u = plt.figure()
    fig_f = plt.figure()

    # traj
    fig_t = plt.figure()
    fig_traj = fig_t.add_subplot(111)
    fig_traj.set_aspect('equal')

    x_1_fig = fig_p.add_subplot(411)
    x_2_fig = fig_p.add_subplot(412)
    x_3_fig = fig_p.add_subplot(413)
    x_4_fig = fig_p.add_subplot(414)

    u_1_fig = fig_u.add_subplot(411)
    u_2_fig = fig_u.add_subplot(412)
    dummy_1_fig = fig_u.add_subplot(413)
    dummy_2_fig = fig_u.add_subplot(414)

    raw_1_fig = fig_f.add_subplot(311)
    raw_2_fig = fig_f.add_subplot(312)
    f_fig = fig_f.add_subplot(313)

    x_1_fig.plot(np.arange(iteration_num) * dt, plant_system.history_x)
    x_1_fig.set_xlabel("time [s]")
    x_1_fig.set_ylabel("x")

    x_2_fig.plot(np.arange(iteration_num) * dt, plant_system.history_y)
    x_2_fig.set_xlabel("time [s]")
    x_2_fig.set_ylabel("y")

    x_3_fig.plot(np.arange(iteration_num) * dt, plant_system.history_yaw)
    x_3_fig.set_xlabel("time [s]")
    x_3_fig.set_ylabel("yaw")

    x_4_fig.plot(np.arange(iteration_num) * dt, plant_system.history_v)
    x_4_fig.set_xlabel("time [s]")
    x_4_fig.set_ylabel("v")

    u_1_fig.plot(np.arange(iteration_num - 1) * dt, controller.history_u_1)
    u_1_fig.set_xlabel("time [s]")
    u_1_fig.set_ylabel("u_a")

    u_2_fig.plot(np.arange(iteration_num - 1) * dt, controller.history_u_2)
    u_2_fig.set_xlabel("time [s]")
    u_2_fig.set_ylabel("u_omega")

    dummy_1_fig.plot(np.arange(iteration_num - 1) *
                     dt, controller.history_dummy_u_1)
    dummy_1_fig.set_xlabel("time [s]")
    dummy_1_fig.set_ylabel("dummy u_1")

    dummy_2_fig.plot(np.arange(iteration_num - 1) *
                     dt, controller.history_dummy_u_2)
    dummy_2_fig.set_xlabel("time [s]")
    dummy_2_fig.set_ylabel("dummy u_2")

    raw_1_fig.plot(np.arange(iteration_num - 1) * dt, controller.history_raw_1)
    raw_1_fig.set_xlabel("time [s]")
    raw_1_fig.set_ylabel("raw_1")

    raw_2_fig.plot(np.arange(iteration_num - 1) * dt, controller.history_raw_2)
    raw_2_fig.set_xlabel("time [s]")
    raw_2_fig.set_ylabel("raw_2")

    f_fig.plot(np.arange(iteration_num - 1) * dt, controller.history_f)
    f_fig.set_xlabel("time [s]")
    f_fig.set_ylabel("optimal error")

    fig_traj.plot(plant_system.history_x,
                  plant_system.history_y, "-r")
    fig_traj.set_xlabel("x [m]")
    fig_traj.set_ylabel("y [m]")
    fig_traj.axis("equal")

    # start state
    plot_car(plant_system.history_x[0],
             plant_system.history_y[0],
             plant_system.history_yaw[0],
             controller.history_u_2[0],
             )

    # goal state
    plot_car(0.0, 0.0, 0.0, 0.0)

    plt.show()


def plot_car(x, y, yaw, steer=0.0, cabcolor="-r", truckcolor="-k"):  # pragma: no cover

    # Vehicle parameters
    LENGTH = 0.4  # [m]
    WIDTH = 0.2  # [m]
    BACKTOWHEEL = 0.1  # [m]
    WHEEL_LEN = 0.03  # [m]
    WHEEL_WIDTH = 0.02  # [m]
    TREAD = 0.07  # [m]

    outline = np.array([[-BACKTOWHEEL, (LENGTH - BACKTOWHEEL), (LENGTH - BACKTOWHEEL),
                         -BACKTOWHEEL, -BACKTOWHEEL],
                        [WIDTH / 2, WIDTH / 2, - WIDTH / 2, -WIDTH / 2, WIDTH / 2]])

    fr_wheel = np.array([[WHEEL_LEN, -WHEEL_LEN, -WHEEL_LEN, WHEEL_LEN, WHEEL_LEN],
                         [-WHEEL_WIDTH - TREAD, -WHEEL_WIDTH - TREAD, WHEEL_WIDTH -
                          TREAD, WHEEL_WIDTH - TREAD, -WHEEL_WIDTH - TREAD]])

    rr_wheel = np.copy(fr_wheel)

    fl_wheel = np.copy(fr_wheel)
    fl_wheel[1, :] *= -1
    rl_wheel = np.copy(rr_wheel)
    rl_wheel[1, :] *= -1

    Rot1 = np.array([[math.cos(yaw), math.sin(yaw)],
                     [-math.sin(yaw), math.cos(yaw)]])
    Rot2 = np.array([[math.cos(steer), math.sin(steer)],
                     [-math.sin(steer), math.cos(steer)]])

    fr_wheel = (fr_wheel.T.dot(Rot2)).T
    fl_wheel = (fl_wheel.T.dot(Rot2)).T
    fr_wheel[0, :] += WB
    fl_wheel[0, :] += WB

    fr_wheel = (fr_wheel.T.dot(Rot1)).T
    fl_wheel = (fl_wheel.T.dot(Rot1)).T

    outline = (outline.T.dot(Rot1)).T
    rr_wheel = (rr_wheel.T.dot(Rot1)).T
    rl_wheel = (rl_wheel.T.dot(Rot1)).T

    outline[0, :] += x
    outline[1, :] += y
    fr_wheel[0, :] += x
    fr_wheel[1, :] += y
    rr_wheel[0, :] += x
    rr_wheel[1, :] += y
    fl_wheel[0, :] += x
    fl_wheel[1, :] += y
    rl_wheel[0, :] += x
    rl_wheel[1, :] += y

    plt.plot(np.array(outline[0, :]).flatten(),
             np.array(outline[1, :]).flatten(), truckcolor)
    plt.plot(np.array(fr_wheel[0, :]).flatten(),
             np.array(fr_wheel[1, :]).flatten(), truckcolor)
    plt.plot(np.array(rr_wheel[0, :]).flatten(),
             np.array(rr_wheel[1, :]).flatten(), truckcolor)
    plt.plot(np.array(fl_wheel[0, :]).flatten(),
             np.array(fl_wheel[1, :]).flatten(), truckcolor)
    plt.plot(np.array(rl_wheel[0, :]).flatten(),
             np.array(rl_wheel[1, :]).flatten(), truckcolor)
    plt.plot(x, y, "*")


def animation(plant, controller, dt):

    skip = 2  # skip index for animation

    for t in range(1, len(controller.history_u_1), skip):
        x = plant.history_x[t]
        y = plant.history_y[t]
        yaw = plant.history_yaw[t]
        v = plant.history_v[t]
        accel = controller.history_u_1[t]
        time = t * dt

        if abs(v) <= 0.01:
            steer = 0.0
        else:
            steer = math.atan2(controller.history_u_2[t] * WB / v, 1.0)

        plt.cla()
        # for stopping simulation with the esc key.
        plt.gcf().canvas.mpl_connect('key_release_event',
                lambda event: [exit(0) if event.key == 'escape' else None])
        plt.plot(plant.history_x, plant.history_y, "-r", label="trajectory")
        plot_car(x, y, yaw, steer=steer)
        plt.axis("equal")
        plt.grid(True)
        plt.title("Time[s]:" + str(round(time, 2)) +
                  ", accel[m/s]:" + str(round(accel, 2)) +
                  ", speed[km/h]:" + str(round(v * 3.6, 2)))
        plt.pause(0.0001)

    plt.close("all")


def main():
    # simulation time
    dt = 0.1
    iteration_time = 150.0  # [s]

    init_x = -4.5
    init_y = -2.5
    init_yaw = math.radians(45.0)
    init_v = -1.0

    # plant
    plant_system = TwoWheeledSystem(
        init_x, init_y, init_yaw, init_v)

    # controller
    controller = NMPCController_with_CGMRES()

    iteration_num = int(iteration_time / dt)
    for i in range(1, iteration_num):
        time = float(i) * dt
        # make input
        u_1s, u_2s = controller.calc_input(
            plant_system.x, plant_system.y, plant_system.yaw, plant_system.v, time)
        # update state
        plant_system.update_state(u_1s[0], u_2s[0])

    if show_animation:  # pragma: no cover
        animation(plant_system, controller, dt)
        plot_figures(plant_system, controller, iteration_num, dt)


if __name__ == "__main__":
    main()
