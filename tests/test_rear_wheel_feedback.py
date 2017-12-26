from unittest import TestCase

import sys
sys.path.append("./PathTracking/rear_wheel_feedback/")

from PathTracking.rear_wheel_feedback import rear_wheel_feedback as m

print(__file__)


class Test(TestCase):

    def test1(self):
        m.show_animation = False
        m.main()
