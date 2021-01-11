from SLAM.EKFSLAM import ekf_slam as m


def test_1():
    m.show_animation = False
    m.main()
