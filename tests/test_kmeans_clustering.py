import conftest
from Mapping.kmeans_clustering import kmeans_clustering as m


def test_1():
    m.show_animation = False
    m.main()


if __name__ == '__main__':
    conftest.run_this_test(__file__)
