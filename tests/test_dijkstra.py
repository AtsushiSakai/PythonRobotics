import conftest  # Add root path to sys.path
from PathPlanning.Dijkstra import dijkstra as m


def test_1():
    m.show_animation = False
    m.main()
