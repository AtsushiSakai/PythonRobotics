from Localization.histogram_filter import histogram_filter as m


def test1():
    m.show_animation = False
    m.SIM_TIME = 1.0
    m.main()
