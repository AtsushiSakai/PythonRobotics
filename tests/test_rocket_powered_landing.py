import conftest  # Add root path to sys.path
import sys

if 'cvxpy' in sys.modules:  # pragma: no cover

    from AerialNavigation.rocket_powered_landing import rocket_powered_landing as m

    def test1():
        m.show_animation = False
        m.main()


if __name__ == '__main__':
    conftest.run_this_test(__file__)
