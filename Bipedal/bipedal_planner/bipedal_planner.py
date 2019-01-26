import numpy as np
import math
from matplotlib import pyplot as plt

class BipedalPlanner(object):
    def __init__(self):
        self.footstep = None
        self.g = 9.8

    def set_footstep(self, footstep):
        self.footstep = footstep

    def inverted_pendulum(self, x, x_dot, px_star, y, y_dot, py_star, z_c, time_width):
        time_split = 100

        for i in range(time_split):
            delta_time = time_width / time_split
            
            x_dot2 = self.g / z_c * (x - px_star)
            x += x_dot * delta_time
            x_dot += x_dot2 * delta_time
            
            y_dot2 = self.g / z_c * (y - py_star)
            y += y_dot * delta_time
            y_dot += y_dot2 * delta_time

            self.com_trajectory.append([x, y])

        return x, x_dot, y, y_dot
        
    def walk(self, T_sup=0.8, z_c=0.8, a=10, b=1, plot=False):
        if self.footstep == None:
            print("No footstep")
            return

        self.com_trajectory = []
        self.footsteps = []
        
        px, py = 0., 0.
        px_star, py_star = px, py
        xi, xi_dot, yi, yi_dot = 0., 0., 0., 0.
        time = 0.
        n = 0
        for i in range(len(self.footstep)):
            # simulate x, y o finverted pendulum
            xi, xi_dot, yi, yi_dot = self.inverted_pendulum(xi, xi_dot, px_star, yi, yi_dot, py_star, z_c, T_sup)
            
            # update time
            time += T_sup
            n += 1

            
            # calculate px, py, x_, y_, vx_, vy_
            f_x, f_y, f_theta = self.footstep[n - 1]
            rotate_mat = np.array([[math.cos(f_theta), -math.sin(f_theta)],
                                   [math.sin(f_theta), math.cos(f_theta)]])
            
            if n == len(self.footstep):
                f_x_next, f_y_next, f_theta_next = 0., 0., 0.
            else:
                f_x_next, f_y_next, f_theta_next = self.footstep[n]                
            rotate_mat_next = np.array([[math.cos(f_theta_next), -math.sin(f_theta_next)],
                                   [math.sin(f_theta_next), math.cos(f_theta_next)]])
            
            T_c = math.sqrt(z_c / self.g)
            C = math.cosh(T_sup / T_c)
            S = math.sinh(T_sup / T_c)
            
            px, py = list(np.array([px, py]) + np.dot(rotate_mat, np.array([f_x, -1 * math.pow(-1, n) * f_y])))
            x_, y_ = list(np.dot(rotate_mat_next, np.array([f_x_next / 2., math.pow(-1, n) * f_y_next / 2.])))
            vx_, vy_ = list(np.dot(rotate_mat_next, np.array([(1 + C) / (T_c * S) * x_, (C - 1) / (T_c * S) * y_])))

            
            # calculate reference COM
            xd, xd_dot = px + x_, vx_
            yd, yd_dot = py + y_, vy_

            # calculate modified footstep
            D = a * math.pow(C - 1, 2) + b * math.pow(S / T_c, 2)
            px_star = -a * (C - 1) / D * (xd - C * xi - T_c * S * xi_dot) - b * S / (T_c * D) * (xd_dot - S / T_c * xi - C * xi_dot)
            py_star = -a * (C - 1) / D * (yd - C * yi - T_c * S * yi_dot) - b * S / (T_c * D) * (yd_dot - S / T_c * yi - C * yi_dot)

            self.footsteps.append([px_star, py_star])

        if plot:
            plt.plot([i[0] for i in self.footsteps], [i[1] for i in self.footsteps])
            plt.plot([i[0] for i in self.com_trajectory], [i[1] for i in self.com_trajectory])
            plt.show()

            

if __name__ == "__main__":
    bipedal_planner = BipedalPlanner()
    
    footstep = [[0.0, 0.2, 0.0],
                [0.3, 0.2, 0.0],
                [0.3, 0.2, 0.0],
                [0.3, 0.2, 0.0],
                [0.0, 0.2, 0.0]]
    bipedal_planner.set_footstep(footstep)
    bipedal_planner.walk(plot=True)
