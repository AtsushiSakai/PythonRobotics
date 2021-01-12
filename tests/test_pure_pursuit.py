import conftest  # Add root path to sys.path
from PathTracking.pure_pursuit import pure_pursuit as m


def test1():
    m.show_animation = False
    m.main()
