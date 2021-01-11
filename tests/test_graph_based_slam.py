from SLAM.GraphBasedSLAM import graph_based_slam as m


def test_1():
    m.show_animation = False
    m.SIM_TIME = 20.0
    m.main()
