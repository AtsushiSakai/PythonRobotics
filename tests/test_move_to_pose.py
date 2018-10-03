from unittest import TestCase

from PathTracking.move_to_pose import move_to_pose as m

print(__file__)


class Test(TestCase):

    def test1(self):
        m.show_animation = False
        m.main()
