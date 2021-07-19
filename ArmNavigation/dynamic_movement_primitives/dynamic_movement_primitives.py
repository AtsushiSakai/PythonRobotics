"""
Author: Jonathan Schwartz (github.com/SchwartzCode)
More information on Dynamic Movement Primitives available at:
https://arxiv.org/abs/2102.03861
https://www.frontiersin.org/articles/10.3389/fncom.2013.00138/full
"""
import matplotlib.pyplot as plt
import numpy as np


class DMP(object):

    def __init__(self, training_data, data_period, K=156.25, B=25,
                 timesteps=500, repel_factor=1):
        """
        Arguments:
            training_data - data in for [(x1,y1), (x2,y2), ...]
            data_period   - amount of time training data covers
            K and B       - spring and damper constants to define DMP behavior
            timesteps     - number of points in generated trajectories
            repel_factor  - controls how much path will avoid obstacles
        """
        self.K = K  # virtual spring constant
        self.B = B  # virtual damper coefficient

        self.dt = data_period / timesteps
        self.timesteps = timesteps

        self.weights = None  # weights used to generate DMP trajectories

        self.repel_factor = repel_factor

        # self.T_orig = data_period
        # self.training_data = training_data

        self.find_basis_functions_weights(training_data, data_period)

    def find_basis_functions_weights(self, training_data, data_period, num_weights=5):
        """
        Arguments:
            data [(steps x spacial dim) np array] - data to replicate with DMP
            data_period [float] - time duration of data
        """

        self.training_data = training_data # for plotting

        dt = data_period / len(training_data)

        init_state = training_data[0]  # initial pos
        goal_state = training_data[-1]  # assume goal is reached by end of data

        # means (C) and std devs (H) of gaussian basis functions
        C = np.linspace(0, 1, num_weights)
        H = (0.65*(1./(num_weights-1))**2)

        for dim in range(len(training_data[0])):

            dimension_data = training_data[:,dim]

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

                Phi = [np.exp(-0.5 * ((i * dt / data_period) - c)**2 / H)
                       for c in C]
                Phi = Phi/np.sum(Phi)

                qdd = (qd - qd_last)/dt

                f = (qdd * data_period**2 - self.K * (g - q) + self.B * qd
                     * data_period) / (g - q0)

                phi_vals.append(Phi)
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

        return self.weights  # TODO: neccesary?

    def recreate_trajectory(self, init_state, goal_state, T):
        """
        init_state - initial state/position
        goal_state - goal state/position
        T  - amount of time to travek q0 -> g
        """

        nrBasis = len(self.weights[0])  # number of gaussian basis functions

        # means (C) and std devs (H) of gaussian basis functions
        C = np.linspace(0, 1, nrBasis)
        H = (0.65*(1./(nrBasis-1))**2)

        # initialize virtual system
        state = init_state
        state_dot = [0,0]
        state_double_dot = [0,0]
        t = 0

        # for plotting
        self.train_t_vals = np.arange(0, T, self.timesteps)
        # TODO: is self.period variable used

        positions = []
        for k in range(self.timesteps):
            new_state = []
            t = t + self.dt

            obs_force = 0
            if self.obstacles is not None:
                obs_force = self.get_obstacle_force(state)

            for dim in range(self.weights.shape[0]):

                q0 = init_state[dim]
                q = state[dim]
                qd = state_dot[dim]
                qdd = state_double_dot[dim]
                g = goal_state[dim]

                if t <= T:
                    Phi = [np.exp(-0.5 * ((t / T) - c)**2 / H) for c in C]
                    Phi = Phi / np.sum(Phi)
                    f = np.dot(Phi, self.weights[dim])
                else:
                    f = 0
                    return np.arange(0, self.timesteps * self.dt, self.dt), positions

                # simulate dynamics
                qdd = self.K*(g-q)/T**2 - self.B*qd/T + (g-q0)*f/T**2 \
                      + obs_force[dim]
                qd = qd + qdd * self.dt
                q = q + qd * self.dt

                state_dot[dim] = qd
                state_double_dot[dim] = qdd

                new_state.append(q)

            state = new_state
            positions.append(new_state)

        return np.arange(0, self.timesteps * self.dt, self.dt), positions

    def get_obstacle_force(self, state):

        obstacle_force = np.zeros(len(self.obstacles[0]))

        for obs in self.obstacles:
            new_force = []

            dist = np.sum(np.sqrt((state - obs)**2))
            force = self.repel_factor * dist

            # TODO: all lists or all np arrays for inputs
            for dim in range(len(self.obstacles[0])):
                new_force.append(force * (state[dim] - obs[dim])**2)

            obstacle_force += np.array(new_force)
            # print(obs, new_force, obstacle_force)

        return obstacle_force

    def solve_trajectory(self, q0, g, T, visualize=True, obstacles=None):

        self.obstacles = obstacles

        t, pos = self.recreate_trajectory(q0, g, T)

        state_hist = np.asarray(pos)

        if visualize:
            if self.training_data.shape[1] == 2:
                if self.obstacles is not None:
                    for obs in self.obstacles:
                        plt.scatter(obs[0], obs[1], color='k')
                plt.plot(self.training_data[:,0], self.training_data[:,1],
                         label="Training Data")
                plt.plot(state_hist[:,0], state_hist[:,1],
                         label="DMP Approximation")
                plt.xlabel("Time [s]")
                plt.ylabel("Position [m]")
                plt.legend()
                plt.show()

        return t, pos

    def show_DMP_purpose(self):
        """
        This function conveys the purpose of DMPs:
            to capture a trajectory and be able to stretch
            and squeeze it in terms of start and stop position
            or time
        """
        q0_orig = self.training_data[0]
        g_orig = self.training_data[-1]

        t_norm, pos_norm = self.recreate_trajectory(q0_orig,
                                                    g_orig,
                                                    self.T_orig)
        t_fast, pos_fast = self.recreate_trajectory(q0_orig,
                                                    g_orig,
                                                    self.T_orig/2)
        t_close, pos_close = self.recreate_trajectory(q0_orig,
                                                      g_orig/2,
                                                      self.T_orig)

        plt.plot(self.train_t_vals, self.training_data,
                 label="Training Data")
        plt.plot(t_norm, pos_norm, label="DMP Approximation",
                 linestyle='--')
        plt.plot(t_fast, pos_fast, label="Decreasing time duration",
                 linestyle='--')
        plt.plot(t_close, pos_close, label='Decreasing goal position',
                 linestyle='--')
        plt.xlabel("X Position")
        plt.ylabel("Y Position")
        plt.legend()
        plt.show()


if __name__ == '__main__':
    period = np.pi / 2
    t = np.arange(0, period, 0.01)
    training_data = np.asarray([np.sin(t) + 0.02*np.random.rand(t.shape[0]),
                                np.cos(t) + 0.02*np.random.rand(t.shape[0]) ]).T

    DMP_controller = DMP(training_data, period)
    # DMP_controller.show_DMP_purpose()

    obs = np.asarray([[0.4,0.8], [1,0.5]])
    DMP_controller.solve_trajectory([0,1], [1,0], 3, obstacles=obs)
