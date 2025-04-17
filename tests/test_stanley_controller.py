import conftest  # Add root path to sys.path
from PathTracking.stanley_controller import stanley_controller as m


def test1():
    m.show_animation = False
    m.main()


if __name__ == '__main__':
    conftest.run_this_test(__file__)
