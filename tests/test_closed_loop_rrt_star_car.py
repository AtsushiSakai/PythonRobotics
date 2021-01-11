import conftest  # Add root path to sys.path
from PathPlanning.ClosedLoopRRTStar import closed_loop_rrt_star_car as m
import random


class Test(TestCase):

    def test1(self):
        random.seed(12345)
        m.show_animation = False
        m.main(gx=1.0, gy=0.0, gyaw=0.0, max_iter=5)
