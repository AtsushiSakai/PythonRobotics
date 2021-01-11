import conftest  # Add root path to sys.path
from Localization.cubature_kalman_filter import cubature_kalman_filter as m


def test1():
    m.show_final = False
    m.show_animation = False
    m.show_ellipse = False
    m.main()
