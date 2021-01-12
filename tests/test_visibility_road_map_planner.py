import conftest  # Add root path to sys.path
from PathPlanning.VoronoiRoadMap import voronoi_road_map as m


def test1():
    m.show_animation = False
    m.main()
