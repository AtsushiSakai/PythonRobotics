"""
Author: Jonathan Schwartz (github.com/SchwartzCode)

This code provides a simple implementation of Dynamic Movement
Primitives, which is an approach to learning curves by modelling
them as a weighted sum of gaussian distributions. This approach
can be used to dampen noise in a curve, and can also be used to
stretch a curve by adjusting its start and end points.

More information on Dynamic Movement Primitives available at:
https://arxiv.org/abs/2102.03861
https://www.frontiersin.org/articles/10.3389/fncom.2013.00138/full

"""


from matplotlib import pyplot as plt
import numpy as np


class DMP(object):

    def __init__(self, training_data, data_period, K=156.25, B=25):
        """
        Arguments:
            training_data - input data of form [N, dim]
            data_period   - amount of time training data covers
            K and B       - spring and damper constants to define
                            DMP behavior
        """

        self.K = K  # virtual spring constant
        self.B = B  # virtual damper coefficient

        self.timesteps = training_data.shape[0]
        self.dt = data_period / self.timesteps

        self.weights = None  # weights used to generate DMP trajectories

        self.T_orig = data_period

        self.training_data = training_data
        self.find_basis_functions_weights(training_data, data_period)

    def find_basis_functions_weights(self, training_data, data_period,
                                     num_weights=10):
        """
        Arguments:
            data [(steps x spacial dim) np array] - data to replicate with DMP
            data_period [float] - time duration of data
        """

        if not isinstance(training_data, np.ndarray):
            print("Warning: you should input training data as an np.ndarray")
        elif training_data.shape[0] < training_data.shape[1]:
            print("Warning: you probably need to transpose your training data")

        dt = data_period / len(training_data)

        init_state = training_data[0]
        goal_state = training_data[-1]

        # means (C) and std devs (H) of gaussian basis functions
        C = np.linspace(0, 1, num_weights)
        H = (0.65*(1./(num_weights-1))**2)

        for dim, _ in enumerate(training_data[0]):

            dimension_data = training_data[:, dim]

            q0 = init_state[dim]
            g = goal_state[dim]

            q = q0
            qd_last = 0

            phi_vals = []
            f_vals = []

            for i, _ in enumerate(dimension_data):
                if i + 1 == len(dimension_data):
                    qd = 0
                else:
                    qd = (dimension_data[i+1] - dimension_data[i]) / dt

                phi = [np.exp(-0.5 * ((i * dt / data_period) - c)**2 / H)
                       for c in C]
                phi = phi/np.sum(phi)

                qdd = (qd - qd_last)/dt

                f = (qdd * data_period**2 - self.K * (g - q) + self.B * qd
                     * data_period) / (g - q0)

                phi_vals.append(phi)
                f_vals.append(f)

                qd_last = qd
                q += qd * dt

            phi_vals = np.asarray(phi_vals)
            f_vals = np.asarray(f_vals)

            w = np.linalg.lstsq(phi_vals, f_vals, rcond=None)

            if self.weights is None:
                self.weights = np.asarray(w[0])
            else:
                self.weights = np.vstack([self.weights, w[0]])

    def recreate_trajectory(self, init_state, goal_state, T):
        """
        init_state - initial state/position
        goal_state - goal state/position
        T  - amount of time to travel q0 -> g
        """

        nrBasis = len(self.weights[0])  # number of gaussian basis functions

        # means (C) and std devs (H) of gaussian basis functions
        C = np.linspace(0, 1, nrBasis)
        H = (0.65*(1./(nrBasis-1))**2)

        # initialize virtual system
        time = 0

        q = init_state
        dimensions = self.weights.shape[0]
        qd = np.zeros(dimensions)

        positions = np.array([])
        for k in range(self.timesteps):
            time = time + self.dt

            qdd = np.zeros(dimensions)

            for dim in range(dimensions):

                if time <= T:
                    phi = [np.exp(-0.5 * ((time / T) - c)**2 / H) for c in C]
                    phi = phi / np.sum(phi)
                    f = np.dot(phi, self.weights[dim])
                else:
                    f = 0

                # simulate dynamics
                qdd[dim] = (self.K*(goal_state[dim] - q[dim])/T**2
                            - self.B*qd[dim]/T
                            + (goal_state[dim] - init_state[dim])*f/T**2)

            qd = qd + qdd * self.dt
            q = q + qd * self.dt

            if positions.size == 0:
                positions = q
            else:
                positions = np.vstack([positions, q])

        t = np.arange(0, self.timesteps * self.dt, self.dt)
        return t, positions

    @staticmethod
    def dist_between(p1, p2):
        return np.linalg.norm(p1 - p2)

    def view_trajectory(self, path, title=None, demo=False):

        path = np.asarray(path)

        plt.cla()
        plt.plot(self.training_data[:, 0], self.training_data[:, 1],
                 label="Training Data")
        plt.plot(path[:, 0], path[:, 1],
                 linewidth=2, label="DMP Approximation")

        plt.xlabel("X Position")
        plt.ylabel("Y Position")
        plt.legend()

        if title is not None:
            plt.title(title)

        if demo:
            plt.xlim([-0.5, 5])
            plt.ylim([-2, 2])
            plt.draw()
            plt.pause(0.02)
        else:
            plt.show()

    def show_DMP_purpose(self):
        """
        This function conveys the purpose of DMPs:
            to capture a trajectory and be able to stretch
            and squeeze it in terms of start and stop position
            or time
        """

        q0_orig = self.training_data[0]
        g_orig = self.training_data[-1]
        T_orig = self.T_orig

        data_range = (np.amax(self.training_data[:, 0])
                      - np.amin(self.training_data[:, 0])) / 4

        q0_right = q0_orig + np.array([data_range, 0])
        q0_up = q0_orig + np.array([0, data_range/2])
        g_left = g_orig - np.array([data_range, 0])
        g_down = g_orig - np.array([0, data_range/2])

        q0_vals = np.vstack([np.linspace(q0_orig, q0_right, 20),
                             np.linspace(q0_orig, q0_up, 20)])
        g_vals = np.vstack([np.linspace(g_orig, g_left, 20),
                            np.linspace(g_orig, g_down, 20)])
        T_vals = np.linspace(T_orig, 2*T_orig, 20)

        for new_q0_value in q0_vals:
            plot_title = "Initial Position = [%s, %s]" % \
                         (round(new_q0_value[0], 2), round(new_q0_value[1], 2))

            _, path = self.recreate_trajectory(new_q0_value, g_orig, T_orig)
            self.view_trajectory(path, title=plot_title, demo=True)

        for new_g_value in g_vals:
            plot_title = "Goal Position = [%s, %s]" % \
                         (round(new_g_value[0], 2), round(new_g_value[1], 2))

            _, path = self.recreate_trajectory(q0_orig, new_g_value, T_orig)
            self.view_trajectory(path, title=plot_title, demo=True)

        for new_T_value in T_vals:
            plot_title = "Period = %s [sec]" % round(new_T_value, 2)

            _, path = self.recreate_trajectory(q0_orig, g_orig, new_T_value)
            self.view_trajectory(path, title=plot_title, demo=True)


def example_DMP():
    """
    Creates a noisy trajectory, fits weights to it, and then adjusts the
    trajectory by moving its start position, goal position, or period
    """
    t = np.arange(0, 3*np.pi/2, 0.01)
    t1 = np.arange(3*np.pi/2, 2*np.pi, 0.01)[:-1]
    t2 = np.arange(0, np.pi/2, 0.01)[:-1]
    t3 = np.arange(np.pi, 3*np.pi/2, 0.01)
    data_x = t + 0.02*np.random.rand(t.shape[0])
    data_y = np.concatenate([np.cos(t1) + 0.1*np.random.rand(t1.shape[0]),
                             np.cos(t2) + 0.1*np.random.rand(t2.shape[0]),
                             np.sin(t3) + 0.1*np.random.rand(t3.shape[0])])
    training_data = np.vstack([data_x, data_y]).T

    period = 3*np.pi/2
    DMP_controller = DMP(training_data, period)

    DMP_controller.show_DMP_purpose()


if __name__ == '__main__':

    example_DMP()
