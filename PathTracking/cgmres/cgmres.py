"""

Nonlinear MPC simulation with CGMRES

author Atsushi Sakai (@Atsushi_twi)

Ref:
- Shunichi09/nonlinear_control: Implementing the nonlinear model predictive control, sliding mode control https://github.com/Shunichi09/nonlinear_control

"""

import numpy as np
import matplotlib.pyplot as plt
import math


def differential_model(yaw, u_1, u_2):

    dx = math.cos(yaw) * u_1
    dy = math.sin(yaw) * u_1
    dyaw = u_2

    return dx, dy, dyaw


class TwoWheeledSystem():

    def __init__(self, init_x, init_y, init_yaw):

        self.x_1 = init_x
        self.x_2 = init_y
        self.x_3 = init_yaw
        self.history_x_1 = [init_x]
        self.history_x_2 = [init_y]
        self.history_x_3 = [init_yaw]

    def update_state(self, u_1, u_2, dt=0.01):

        dx, dy, dyaw = differential_model(self.x_3, u_1, u_2)

        self.x_1 += dt * dx
        self.x_2 += dt * dy
        self.x_3 += dt * dyaw

        # save
        self.history_x_1.append(self.x_1)
        self.history_x_2.append(self.x_2)
        self.history_x_3.append(self.x_3)


class NMPCSimulatorSystem():

    def calc_predict_and_adjoint_state(self, x_1, x_2, x_3, u_1s, u_2s, N, dt):
        x_1s, x_2s, x_3s = self._calc_predict_states(
            x_1, x_2, x_3, u_1s, u_2s, N, dt)  # by using state equation
        lam_1s, lam_2s, lam_3s = self._calc_adjoint_states(
            x_1s, x_2s, x_3s, u_1s, u_2s, N, dt)  # by using adjoint equation

        return x_1s, x_2s, x_3s, lam_1s, lam_2s, lam_3s

    def _calc_predict_states(self, x_1, x_2, x_3, u_1s, u_2s, N, dt):
        x_1s = [x_1]
        x_2s = [x_2]
        x_3s = [x_3]

        for i in range(N):
            temp_x_1, temp_x_2, temp_x_3 = self._predict_state_with_oylar(
                x_1s[i], x_2s[i], x_3s[i], u_1s[i], u_2s[i], dt)
            x_1s.append(temp_x_1)
            x_2s.append(temp_x_2)
            x_3s.append(temp_x_3)

        return x_1s, x_2s, x_3s

    def _calc_adjoint_states(self, x_1s, x_2s, x_3s, u_1s, u_2s, N, dt):
        lam_1s = [x_1s[-1]]
        lam_2s = [x_2s[-1]]
        lam_3s = [x_3s[-1]]

        # backward adjoint state calc
        for i in range(N - 1, 0, -1):
            temp_lam_1, temp_lam_2, temp_lam_3 = self._adjoint_state_with_oylar(
                x_1s[i], x_2s[i], x_3s[i], lam_1s[0], lam_2s[0], lam_3s[0], u_1s[i], u_2s[i], dt)
            lam_1s.insert(0, temp_lam_1)
            lam_2s.insert(0, temp_lam_2)
            lam_3s.insert(0, temp_lam_3)

        return lam_1s, lam_2s, lam_3s

    def final_state_func(self):
        """this func usually need
        """
        pass

    def _predict_state_with_oylar(self, x_1, x_2, x_3, u_1, u_2, dt):

        dx, dy, dyaw = differential_model(
            x_3, u_1, u_2)

        next_x_1 = x_1 + dt * dx
        next_x_2 = x_2 + dt * dy
        next_x_3 = x_3 + dt * dyaw

        return next_x_1, next_x_2, next_x_3

    def _adjoint_state_with_oylar(self, x_1, x_2, x_3, lam_1, lam_2, lam_3, u_1, u_2, dt):

        # ∂H/∂x
        pre_lam_1 = lam_1 + dt * 0.0
        pre_lam_2 = lam_2 + dt * 0.0
        pre_lam_3 = lam_3 + dt * \
            (- lam_1 * math.sin(x_3) * u_1 + lam_2 * math.cos(x_3) * u_1)

        return pre_lam_1, pre_lam_2, pre_lam_3


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
        self.tf = 1.  # final time
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

    def calc_input(self, x_1, x_2, x_3, time):

        # calculating sampling time
        dt = self.tf * (1. - np.exp(-self.alpha * time)) / float(self.N)

        # x_dot
        x_1_dot, x_2_dot, x_3_dot = differential_model(
            x_3, self.u_1s[0], self.u_2s[0])

        dx_1 = x_1_dot * self.ht
        dx_2 = x_2_dot * self.ht
        dx_3 = x_3_dot * self.ht

        x_1s, x_2s, x_3s, lam_1s, lam_2s, lam_3s = self.simulator.calc_predict_and_adjoint_state(
            x_1 + dx_1, x_2 + dx_2, x_3 + dx_3, self.u_1s, self.u_2s, self.N, dt)

        # Fxt:F(U,x+hx˙,t+h)
        Fxt = self._calc_f(x_1s, x_2s, x_3s, lam_1s, lam_2s, lam_3s,
                           self.u_1s, self.u_2s, self.dummy_u_1s, self.dummy_u_2s,
                           self.raw_1s, self.raw_2s, self.N, dt)

        # F:F(U,x,t)
        x_1s, x_2s, x_3s, lam_1s, lam_2s, lam_3s = self.simulator.calc_predict_and_adjoint_state(
            x_1, x_2, x_3, self.u_1s, self.u_2s, self.N, dt)

        F = self._calc_f(x_1s, x_2s, x_3s, lam_1s, lam_2s, lam_3s,
                         self.u_1s, self.u_2s, self.dummy_u_1s, self.dummy_u_2s,
                         self.raw_1s, self.raw_2s, self.N, dt)

        right = -self.zeta * F - ((Fxt - F) / self.ht)

        du_1 = self.u_1s * self.ht
        du_2 = self.u_2s * self.ht
        ddummy_u_1 = self.dummy_u_1s * self.ht
        ddummy_u_2 = self.dummy_u_2s * self.ht
        draw_1 = self.raw_1s * self.ht
        draw_2 = self.raw_2s * self.ht

        x_1s, x_2s, x_3s, lam_1s, lam_2s, lam_3s = self.simulator.calc_predict_and_adjoint_state(
            x_1 + dx_1, x_2 + dx_2, x_3 + dx_3, self.u_1s + du_1, self.u_2s + du_2, self.N, dt)

        # Fuxt:F(U+hdU(0),x+hx˙,t+h)
        Fuxt = self._calc_f(x_1s, x_2s, x_3s, lam_1s, lam_2s, lam_3s,
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

        for i in range(self.max_iteration):
            du_1 = vs[::self.input_num, i] * self.ht
            du_2 = vs[1::self.input_num, i] * self.ht
            ddummy_u_1 = vs[2::self.input_num, i] * self.ht
            ddummy_u_2 = vs[3::self.input_num, i] * self.ht
            draw_1 = vs[4::self.input_num, i] * self.ht
            draw_2 = vs[5::self.input_num, i] * self.ht

            x_1s, x_2s, x_3s, lam_1s, lam_2s, lam_3s = self.simulator.calc_predict_and_adjoint_state(
                x_1 + dx_1, x_2 + dx_2, x_3 + dx_3, self.u_1s + du_1, self.u_2s + du_2, self.N, dt)

            Fuxt = self._calc_f(x_1s, x_2s, x_3s, lam_1s, lam_2s, lam_3s,
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

        x_1s, x_2s, x_3s, lam_1s, lam_2s, lam_3s = self.simulator.calc_predict_and_adjoint_state(
            x_1, x_2, x_3, self.u_1s, self.u_2s, self.N, dt)

        F = self._calc_f(x_1s, x_2s, x_3s, lam_1s, lam_2s, lam_3s,
                         self.u_1s, self.u_2s, self.dummy_u_1s, self.dummy_u_2s,
                         self.raw_1s, self.raw_2s, self.N, dt)

        print("check F = {0}".format(np.linalg.norm(F)))

        # for save
        self.history_f.append(np.linalg.norm(F))
        self.history_u_1.append(self.u_1s[0])
        self.history_u_2.append(self.u_2s[0])
        self.history_dummy_u_1.append(self.dummy_u_1s[0])
        self.history_dummy_u_2.append(self.dummy_u_2s[0])
        self.history_raw_1.append(self.raw_1s[0])
        self.history_raw_2.append(self.raw_2s[0])

        return self.u_1s, self.u_2s

    def _calc_f(self, x_1s, x_2s, x_3s, lam_1s, lam_2s, lam_3s,
                u_1s, u_2s, dummy_u_1s, dummy_u_2s, raw_1s, raw_2s, N, dt):

        F = []
        for i in range(N):
            # ∂H/∂u(xi, ui, λi)
            F.append(u_1s[i] + lam_1s[i] * math.cos(x_3s[i]) +
                     lam_2s[i] * math.sin(x_3s[i]) + 2 * raw_1s[i] * u_1s[i])
            F.append(u_2s[i] + lam_3s[i] + 2 * raw_2s[i] * u_2s[i])
            F.append(-0.01 + 2. * raw_1s[i] * dummy_u_1s[i])
            F.append(-0.01 + 2. * raw_2s[i] * dummy_u_2s[i])

            # C(xi, ui, λi)
            F.append(u_1s[i]**2 + dummy_u_1s[i]**2 - 1.**2)
            F.append(u_2s[i]**2 + dummy_u_2s[i]**2 - 1.5**2)

        return np.array(F)


def plot_figures(plant_system, controller, iteration_num, dt):
    # figure
    # time history
    fig_p = plt.figure()
    fig_u = plt.figure()
    fig_f = plt.figure()

    # traj
    fig_t = plt.figure()
    fig_traj = fig_t.add_subplot(111)
    fig_traj.set_aspect('equal')

    x_1_fig = fig_p.add_subplot(311)
    x_2_fig = fig_p.add_subplot(312)
    x_3_fig = fig_p.add_subplot(313)

    u_1_fig = fig_u.add_subplot(411)
    u_2_fig = fig_u.add_subplot(412)
    dummy_1_fig = fig_u.add_subplot(413)
    dummy_2_fig = fig_u.add_subplot(414)

    raw_1_fig = fig_f.add_subplot(311)
    raw_2_fig = fig_f.add_subplot(312)
    f_fig = fig_f.add_subplot(313)

    x_1_fig.plot(np.arange(iteration_num) * dt, plant_system.history_x_1)
    x_1_fig.set_xlabel("time [s]")
    x_1_fig.set_ylabel("x_1")

    x_2_fig.plot(np.arange(iteration_num) * dt, plant_system.history_x_2)
    x_2_fig.set_xlabel("time [s]")
    x_2_fig.set_ylabel("x_2")

    x_3_fig.plot(np.arange(iteration_num) * dt, plant_system.history_x_3)
    x_3_fig.set_xlabel("time [s]")
    x_3_fig.set_ylabel("x_3")

    u_1_fig.plot(np.arange(iteration_num - 1) * dt, controller.history_u_1)
    u_1_fig.set_xlabel("time [s]")
    u_1_fig.set_ylabel("u_v")

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

    fig_traj.plot(plant_system.history_x_1,
                  plant_system.history_x_2, color="b", linestyle="dashed")
    fig_traj.set_xlabel("x [m]")
    fig_traj.set_ylabel("y [m]")

    plt.show()


def main():
    # simulation time
    dt = 0.01
    iteration_time = 15.0  # [s]

    init_x = -4.5
    init_y = -2.5
    init_yaw = math.radians(45.0)

    # plant
    plant_system = TwoWheeledSystem(
        init_x, init_y, init_yaw)

    # controller
    controller = NMPCController_with_CGMRES()

    iteration_num = int(iteration_time / dt)
    for i in range(1, iteration_num):
        time = float(i) * dt
        # make input
        u_1s, u_2s = controller.calc_input(
            plant_system.x_1, plant_system.x_2, plant_system.x_3, time)
        # update state
        plant_system.update_state(u_1s[0], u_2s[0])

    plot_figures(plant_system, controller, iteration_num, dt)


if __name__ == "__main__":
    main()
