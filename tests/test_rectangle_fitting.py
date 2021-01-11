import conftest  # Add root path to sys.path
from Mapping.rectangle_fitting import rectangle_fitting as m


def test1():
    m.show_animation = False
    m.main()
