import numpy as np
import math
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.pyplot as plt

class Link:
    def __init__(self, dh_params):
        self.dh_params_ = dh_params

    def calc_transformation_matrix(self):
        theta = self.dh_params_[0]
        alpha = self.dh_params_[1]
        a = self.dh_params_[2]
        d = self.dh_params_[3]

        trans = np.array(
            [[math.cos(theta), -math.sin(theta), 0, a],
             [math.cos(alpha) * math.sin(theta), math.cos(alpha) * math.cos(theta), -math.sin(alpha), -d * math.sin(alpha)],
             [math.sin(alpha) * math.sin(theta), math.sin(alpha) * math.cos(theta), math.cos(alpha), d * math.cos(alpha)],
             [0, 0, 0, 1]])

        return trans

class NLinkArm:
    def __init__(self, dh_params_list):
        self.link_list = []
        for i in range(len(dh_params_list)):
            self.link_list.append(Link(dh_params_list[i]))

        self.fig = plt.figure()
        self.ax = Axes3D(self.fig)

    def calc_transformation_matrix(self):
        trans = np.array([[1, 0, 0, 0],
                             [0, 1, 0, 0],
                             [0, 0, 1, 0],
                             [0, 0, 0, 1]])
        for i in range(len(self.link_list)):
            trans = np.dot(trans, self.link_list[i].calc_transformation_matrix())
            
        return trans
    
    def forward_kinematics(self):
        trans = self.calc_transformation_matrix()

        x = trans[0, 3]
        y = trans[1, 3]
        z = trans[2, 3]
        alpha = math.atan2(trans[1, 2], trans[1, 3])
        beta = math.atan2(trans[0, 2] * math.cos(alpha) + trans[1, 2] * math.sin(alpha), trans[2, 2])
        gamma = math.atan2(-trans[0, 0] * math.sin(alpha) + trans[1, 0] * math.cos(alpha), -trans[0, 1] * math.sin(alpha) + trans[1, 1] * math.cos(alpha))

        return [x, y, z, alpha, beta, gamma]

    def set_joint_angles(self, joint_angle_list):
        for i in range(len(self.link_list)):
            self.link_list[i].dh_params_[0] = joint_angle_list[i]
        
    def plot(self):
        x_list = []
        y_list = []
        z_list = []

        trans = np.identity(4)
        
        x_list.append(trans[0, 3])
        y_list.append(trans[1, 3])
        z_list.append(trans[2, 3])
        for i in range(len(self.link_list)):
            trans = np.dot(trans, self.link_list[i].calc_transformation_matrix())
            x_list.append(trans[0, 3])
            y_list.append(trans[1, 3])
            z_list.append(trans[2, 3])
            
        self.ax.plot(x_list, y_list, z_list, "o-", color="#00aa00", ms=4, mew=0.5)
        self.ax.plot([0], [0], [0], "o")
        
        self.ax.set_xlim(-1, 1)
        self.ax.set_ylim(-1, 1)
        self.ax.set_zlim(-1, 1)        
        plt.show()
        
if __name__ == "__main__":
    n_link_arm = NLinkArm([[0., -math.pi/2, .1, 0.],
                           [math.pi/2, math.pi/2, 0., 0.],
                           [0., -math.pi/2, 0., .4],
                           [0., math.pi/2, 0., 0.],
                           [0., -math.pi/2, 0., .321],
                           [0., math.pi/2, 0., 0.],
                           [0., 0., 0., 0.]])

    print(n_link_arm.forward_kinematics())
    n_link_arm.set_joint_angles([1, 1, 1, 1, 1, 1, 1])
    n_link_arm.plot()
