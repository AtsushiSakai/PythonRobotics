from unittest import TestCase
from PathTracking.pure_pursuit import pure_pursuit as m

print("pure_pursuit test")


class Test(TestCase):

    def test1(self):
        m.show_animation = False
        m.main()
