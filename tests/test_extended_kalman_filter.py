from Localization.extended_kalman_filter import extended_kalman_filter as m


def test_1():
    m.show_animation = False
    m.main()
