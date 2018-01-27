from unittest import TestCase

from Localization.unscented_kalman_filter import unscented_kalman_filter as m

print(__file__)


class Test(TestCase):

    def test1(self):
        m.show_animation = False
        m.main()
