import conftest  # Add root path to sys.path
from Mapping.raycasting_grid_map import raycasting_grid_map as m


def test1():
    m.show_animation = False
    m.main()
