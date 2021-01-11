import conftest  # Add root path to sys.path
from PathPlanning.ProbabilisticRoadMap import probabilistic_road_map


def test1():
    probabilistic_road_map.show_animation = False
    probabilistic_road_map.main()


if __name__ == '__main__':
    conftest.run_this_test(__file__)
