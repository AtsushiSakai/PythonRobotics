from PathPlanning.BatchInformedRRTStar import batch_informed_rrtstar as m
import random


def test_1():
    m.show_animation = False
    random.seed(12345)
    m.main(maxIter=10)
