import conftest
from Localization.histogram_filter import histogram_filter as m


def test1():
    m.show_animation = False
    m.SIM_TIME = 1.0
    m.main()


if __name__ == '__main__':
    conftest.run_this_test(__file__)
