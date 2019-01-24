import math
from NLinkArm import NLinkArm
import random
import time

def random_val(min_val, max_val):
    return min_val + random.random() * (max_val - min_val)

for i in range(10):
    n_link_arm = NLinkArm([[0., -math.pi/2, .1, 0.],
                           [math.pi/2, math.pi/2, 0., 0.],
                           [0., -math.pi/2, 0., .4],
                           [0., math.pi/2, 0., 0.],
                           [0., -math.pi/2, 0., .321],
                           [0., math.pi/2, 0., 0.],
                           [0., 0., 0., 0.]])

    n_link_arm.set_joint_angles([random_val(-1, 1) for j in range(len(n_link_arm.link_list))])
    n_link_arm.plot()

