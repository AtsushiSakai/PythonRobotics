from unittest import TestCase

from Localization.extended_kalman_filter import extended_kalman_filter as m

print(__file__)


class Test(TestCase):

    def test1(self):
        m.show_animation = False
        m.main()
