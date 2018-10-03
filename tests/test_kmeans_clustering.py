from unittest import TestCase

from Mapping.kmeans_clustering import kmeans_clustering as m

print(__file__)


class Test(TestCase):

    def test1(self):
        m.show_animation = False
        m.main()
