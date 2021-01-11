import conftest  # Add root path to sys.path
from PathTracking.cgmres_nmpc import cgmres_nmpc as m


def test1():
    m.show_animation = False
    m.main()
