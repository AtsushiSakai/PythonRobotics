import matplotlib.pyplot as plt
import numpy as np

class DMP(object):

    def __init__(self, K=156.25, B=25, dt=0.001, timesteps=5000, training_data=None):
        self.K = K  # virtual spring constant
        self.B = B  # virtual damper coefficient

        self.dt = dt
        self.timesteps = timesteps

        if training_data is None:
            self.training_data = []
            self.train_t_vals = np.linspace(0, np.pi, 250)

            for t in self.train_t_vals[:-125]:
                self.training_data.append(np.random.normal(np.sin(t), 0.02))
            for t in self.train_t_vals[125:]:
                self.training_data.append(np.random.normal(self.training_data[-1], 0.005))

            dt = np.max(self.train_t_vals)/(self.train_t_vals.size - 1)
            self.find_basis_functions_weights(training_data, dt)
        else:
            raise NotImplementedError

    def find_basis_functions_weights(self, data, dt):
        # need to do linear regression to get w

        self.w = [-275, 405.7, -535, 264, -111]

        return 0

    def recreate_trajectory(self, q0, g, T):
        '''
        q0 - initial state/position
        g  - goal state/position
        T  - amount of time to travek q0 -> g
        '''

        nrBasis = len(self.w)  # number of gaussian basis functions

        # means (C) and std devs (H) of gaussian basis functions
        C = np.linspace(0,1,nrBasis)
        H = (0.65*(1./(nrBasis-1))**2)

        # initialize virtual system
        q = q0
        qd = 0
        qdd = 0
        t = 0

        positions = []
        for k in range(self.timesteps):
            t = t + self.dt
            if t <=T:
                Phi = [np.exp(-0.5 * ((t/T)-c)**2 / H) for c in C]
                Phi = Phi/np.sum(Phi)
                f = np.dot(Phi, self.w)
            else:
                f=0

            # simulate dynamics
            qdd = self.K*(g-q)/T**2 - self.B*qd/T + (g-q0)*f/T**2
            qd = qd + qdd * self.dt
            q = q + qd * self.dt
            positions.append(q)

        return np.arange(0,self.timesteps * self.dt, self.dt), positions

    def solve_trajectory(self, q0, g, T):

        t, pos = self.recreate_trajectory(q0, g, T)

        plt.plot(self.train_t_vals, self.training_data, label="Training Data")
        plt.plot(t,pos,label="DMP Approximation")
        plt.xlabel("Time [s]")
        plt.ylabel("Position [m]")
        plt.legend()
        plt.show()


if __name__ == '__main__':
    DMP_controller = DMP()
    DMP_controller.solve_trajectory(-1, 3,5)
