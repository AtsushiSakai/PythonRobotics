"""
Inverse Kinematics for an n-link arm in 3D
Author: Takayuki Murooka (takayuki5168)
"""
import math
from NLinkArm import NLinkArm
import random


def random_val(min_val, max_val):
    return min_val + random.random() * (max_val - min_val)

if __name__ == "__main__":
    print("Start solving Inverse Kinematics 10 times")

    # init NLinkArm with Denavit-Hartenberg parameters of PR2        
    n_link_arm = NLinkArm([[0., -math.pi/2, .1, 0.],
                           [math.pi/2, math.pi/2, 0., 0.],
                           [0., -math.pi/2, 0., .4],
                           [0., math.pi/2, 0., 0.],
                           [0., -math.pi/2, 0., .321],
                           [0., math.pi/2, 0., 0.],
                           [0., 0., 0., 0.]])

    # execute IK 10 times
    for i in range(10):
        n_link_arm.inverse_kinematics([random_val(-0.5, 0.5),
                                       random_val(-0.5, 0.5),
                                       random_val(-0.5, 0.5),
                                       random_val(-0.5, 0.5),
                                       random_val(-0.5, 0.5),
                                       random_val(-0.5, 0.5)], plot=True)
