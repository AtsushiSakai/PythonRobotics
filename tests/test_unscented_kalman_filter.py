import conftest  # Add root path to sys.path
from Localization.unscented_kalman_filter import unscented_kalman_filter as m


def test1():
    m.show_animation = False
    m.main()
