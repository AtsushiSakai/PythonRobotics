import conftest
from Mapping.gaussian_grid_map import gaussian_grid_map as m


def test1():
    m.show_animation = False
    m.main()


if __name__ == '__main__':
    conftest.run_this_test(__file__)
