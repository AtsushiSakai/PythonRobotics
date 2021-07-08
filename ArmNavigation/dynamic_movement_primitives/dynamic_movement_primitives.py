'''
Author: Jonathan Schwartz (github.com/SchwartzCode)
More information on Dynamic Movement Primitives available at:
https://arxiv.org/abs/2102.03861
https://www.frontiersin.org/articles/10.3389/fncom.2013.00138/full
'''
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
            self.train_t_vals = np.linspace(0, np.pi, 150)

            for t in self.train_t_vals[:125]:
                self.training_data.append(np.random.normal(np.sin(t), 0.02))
            for t in self.train_t_vals[125:]:
                self.training_data.append(self.training_data[-1])

            dt = np.max(self.train_t_vals)/(self.train_t_vals.size - 1)
        else:
            self.train_t_vals = training_data[0]
            self.training_data = training_data[1]
            dt = self.train_t_vals[1] - self.train_t_vals[0]

        self.find_basis_functions_weights(self.training_data, dt)

    def find_basis_functions_weights(self, data, dt, num_weights=5):

        # means (C) and std devs (H) of gaussian basis functions
        C = np.linspace(0,1,num_weights)
        H = (0.65*(1./(num_weights-1))**2)

        q0 = data[0]  # initial pos
        g  = data[-1] # assume goal is reached by end of data
        self.T_orig = len(data)*dt  # time duration of data

        q = q0
        qd_last = 0

        phi_vals = []
        f_vals = []

        for i in range(len(data)):
            if i+1 == len(data):
                qd = 0
            else:
                qd = (data[i+1] - data[i])/dt

            Phi = [np.exp(-0.5 * ((i*dt/self.T_orig)-c)**2 / H) for c in C]
            Phi = Phi/np.sum(Phi)

            qdd = (qd - qd_last)/dt

            f = (qdd * self.T_orig**2 - self.K*(g - q) + self.B*qd*self.T_orig) / (g - q0)

            phi_vals.append(Phi)
            f_vals.append(f)

            qd_last = qd
            q += qd*dt

        phi_vals = np.asarray(phi_vals)
        f_vals = np.asarray(f_vals)

        print(phi_vals.shape, f_vals.shape)
        w = np.linalg.lstsq(phi_vals, f_vals, rcond=None)
        self.w = w[0]

        return w[0]

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

    def solve_trajectory(self, q0, g, T, visualize=True):

        t, pos = self.recreate_trajectory(q0, g, T)

        if visualize:
            plt.plot(self.train_t_vals, self.training_data, label="Training Data")
            plt.plot(t,pos,label="DMP Approximation")
            plt.xlabel("Time [s]")
            plt.ylabel("Position [m]")
            plt.legend()
            plt.show()

        return t, pos

    def show_DMP_purpose(self):
        '''
        This function conveys the purpose of DMPs:
            to capture a trajectory and be able to stretch
            and squeeze it in terms of start and stop position
            or time
        '''
        q0_orig = self.training_data[0]
        g_orig = self.training_data[-1]

        t_norm, pos_norm = self.recreate_trajectory(q0_orig, g_orig, self.T_orig)
        t_fast, pos_fast = self.recreate_trajectory(q0_orig, g_orig, self.T_orig/2)
        t_close, pos_close = self.recreate_trajectory(q0_orig, g_orig/2, self.T_orig)

        plt.plot(self.train_t_vals, self.training_data, label="Training Data")
        plt.plot(t_norm,pos_norm,label="DMP Approximation", linestyle='--')
        plt.plot(t_fast, pos_fast, label="Decreasing time duration", linestyle='--')
        plt.plot(t_close, pos_close, label='Decreasing goal position', linestyle='--')
        plt.xlabel("Time [s]")
        plt.ylabel("Position [m]")
        plt.legend()
        plt.show()

if __name__ == '__main__':
    DMP_controller = DMP()
    # DMP_controller.show_DMP_purpose()
    # DMP_controller.solve_trajectory(0, 1, 3)
