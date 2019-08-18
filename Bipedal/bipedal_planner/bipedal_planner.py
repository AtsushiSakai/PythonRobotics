"""
Bipedal Walking with modifying designated footsteps
author: Takayuki Murooka (takayuki5168)
"""
import numpy as np
import math
from matplotlib import pyplot as plt
import matplotlib.patches as pat
from mpl_toolkits.mplot3d import Axes3D
import mpl_toolkits.mplot3d.art3d as art3d


class BipedalPlanner(object):
    def __init__(self):
        self.ref_footsteps = None
        self.g = 9.8

    def set_ref_footsteps(self, ref_footsteps):
        self.ref_footsteps = ref_footsteps

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

            if i % 10 == 0:
                self.com_trajectory.append([x, y])

        return x, x_dot, y, y_dot

    def walk(self, T_sup=0.8, z_c=0.8, a=10, b=1, plot=False):
        if self.ref_footsteps is None:
            print("No footsteps")
            return

        # set up plotter
        if plot:
            fig = plt.figure()
            ax = Axes3D(fig)
            com_trajectory_for_plot = []

        self.com_trajectory = []
        self.ref_p = []   # reference footstep positions
        self.act_p = []   # actual footstep positions

        px, py = 0.0, 0.0   # reference footstep position
        px_star, py_star = px, py   # modified footstep position
        xi, xi_dot, yi, yi_dot = 0.0, 0.0, 0.01, 0.0
        time = 0.0
        n = 0
        self.ref_p.append([px, py, 0])
        self.act_p.append([px, py, 0])
        for i in range(len(self.ref_footsteps)):
            # simulate x, y and those of dot of inverted pendulum
            xi, xi_dot, yi, yi_dot = self.inverted_pendulum(
                xi, xi_dot, px_star, yi, yi_dot, py_star, z_c, T_sup)

            # update time
            time += T_sup
            n += 1

            # calculate px, py, x_, y_, vx_, vy_
            f_x, f_y, f_theta = self.ref_footsteps[n - 1]
            rotate_mat = np.array([[math.cos(f_theta), -math.sin(f_theta)],
                                   [math.sin(f_theta), math.cos(f_theta)]])

            if n == len(self.ref_footsteps):
                f_x_next, f_y_next, f_theta_next = 0., 0., 0.
            else:
                f_x_next, f_y_next, f_theta_next = self.ref_footsteps[n]
            rotate_mat_next = np.array([[math.cos(f_theta_next), -math.sin(f_theta_next)],
                                        [math.sin(f_theta_next), math.cos(f_theta_next)]])

            T_c = math.sqrt(z_c / self.g)
            C = math.cosh(T_sup / T_c)
            S = math.sinh(T_sup / T_c)

            px, py = list(np.array(
                [px, py]) + np.dot(rotate_mat, np.array([f_x, -1 * math.pow(-1, n) * f_y])))
            x_, y_ = list(np.dot(rotate_mat_next, np.array(
                [f_x_next / 2., math.pow(-1, n) * f_y_next / 2.])))
            vx_, vy_ = list(np.dot(rotate_mat_next, np.array(
                [(1 + C) / (T_c * S) * x_, (C - 1) / (T_c * S) * y_])))
            self.ref_p.append([px, py, f_theta])

            # calculate reference COM
            xd, xd_dot = px + x_, vx_
            yd, yd_dot = py + y_, vy_

            # calculate modified footsteps
            D = a * math.pow(C - 1, 2) + b * math.pow(S / T_c, 2)
            px_star = -a * (C - 1) / D * (xd - C * xi - T_c * S * xi_dot) - \
                b * S / (T_c * D) * (xd_dot - S / T_c * xi - C * xi_dot)
            py_star = -a * (C - 1) / D * (yd - C * yi - T_c * S * yi_dot) - \
                b * S / (T_c * D) * (yd_dot - S / T_c * yi - C * yi_dot)
            self.act_p.append([px_star, py_star, f_theta])

            # plot
            if plot:
                # for plot trajectory, plot in for loop
                for c in range(len(self.com_trajectory)):
                    if c > len(com_trajectory_for_plot):
                        # set up plotter
                        plt.cla()
                        ax.set_zlim(0, z_c * 2)
                        ax.set_aspect('equal', 'datalim')

                        # update com_trajectory_for_plot
                        com_trajectory_for_plot.append(self.com_trajectory[c])

                        # plot com
                        ax.plot([p[0] for p in com_trajectory_for_plot], [p[1] for p in com_trajectory_for_plot], [
                                0 for p in com_trajectory_for_plot], color="red")

                        # plot inverted pendulum
                        ax.plot([px_star, com_trajectory_for_plot[-1][0]],
                                [py_star, com_trajectory_for_plot[-1][1]],
                                [0, z_c], color="green", linewidth=3)
                        ax.scatter([com_trajectory_for_plot[-1][0]],
                                   [com_trajectory_for_plot[-1][1]],
                                   [z_c], color="green", s=300)

                        # foot rectangle for self.ref_p
                        foot_width = 0.06
                        foot_height = 0.04
                        for j in range(len(self.ref_p)):
                            angle = self.ref_p[j][2] + \
                                math.atan2(foot_height, foot_width) - math.pi
                            r = math.sqrt(
                                math.pow(foot_width / 3., 2) + math.pow(foot_height / 2., 2))
                            rec = pat.Rectangle(xy=(self.ref_p[j][0] + r * math.cos(angle), self.ref_p[j][1] + r * math.sin(angle)),
                                                width=foot_width, height=foot_height, angle=self.ref_p[j][2] * 180 / math.pi, color="blue", fill=False, ls=":")
                            ax.add_patch(rec)
                            art3d.pathpatch_2d_to_3d(rec, z=0, zdir="z")

                        # foot rectangle for self.act_p
                        for j in range(len(self.act_p)):
                            angle = self.act_p[j][2] + \
                                math.atan2(foot_height, foot_width) - math.pi
                            r = math.sqrt(
                                math.pow(foot_width / 3., 2) + math.pow(foot_height / 2., 2))
                            rec = pat.Rectangle(xy=(self.act_p[j][0] + r * math.cos(angle), self.act_p[j][1] + r * math.sin(angle)),
                                                width=foot_width, height=foot_height, angle=self.act_p[j][2] * 180 / math.pi, color="blue", fill=False)
                            ax.add_patch(rec)
                            art3d.pathpatch_2d_to_3d(rec, z=0, zdir="z")

                        plt.draw()
                        plt.pause(0.001)
        if plot:
            plt.show()


if __name__ == "__main__":
    bipedal_planner = BipedalPlanner()

    footsteps = [[0.0, 0.2, 0.0],
                 [0.3, 0.2, 0.0],
                 [0.3, 0.2, 0.2],
                 [0.3, 0.2, 0.2],
                 [0.0, 0.2, 0.2]]
    bipedal_planner.set_ref_footsteps(footsteps)
    bipedal_planner.walk(plot=True)
