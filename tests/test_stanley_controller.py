from unittest import TestCase

import sys
sys.path.append("./PathTracking/stanley_controller/")

from PathTracking.stanley_controller import stanley_controller as m

print("stanley controller test")


class Test(TestCase):

    def test1(self):
        m.show_animation = False
        m.main()
