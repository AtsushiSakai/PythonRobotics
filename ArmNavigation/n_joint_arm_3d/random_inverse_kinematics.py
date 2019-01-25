import math
from NLinkArm import NLinkArm
import random
import time


n_link_arm = NLinkArm([[0., -math.pi/2, .1, 0.],
                       [math.pi/2, math.pi/2, 0., 0.],
                       [0., -math.pi/2, 0., .4],
                       [0., math.pi/2, 0., 0.],
                       [0., -math.pi/2, 0., .321],
                       [0., math.pi/2, 0., 0.],
                       [0., 0., 0., 0.]])

#n_link_arm.inverse_kinematics([-0.621, 0., 0., 0., 0., math.pi / 2])
n_link_arm.inverse_kinematics([-0.5, 0., 0.1, 0., 0., math.pi / 2])
n_link_arm.plot()
