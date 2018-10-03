from unittest import TestCase

from Localization.histogram_filter import histogram_filter as m

print(__file__)


class Test(TestCase):

    def test1(self):
        m.show_animation = False
        m.main()
