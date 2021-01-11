import conftest
from PathTracking.cgmres_nmpc import cgmres_nmpc as m


def test1():
    m.show_animation = False
    m.main()


if __name__ == '__main__':
    conftest.run_this_test(__file__)
