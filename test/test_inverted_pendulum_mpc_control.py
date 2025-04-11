import conftest

from InvertedPendulum import inverted_pendulum_mpc_control as m


def test1():
    m.show_animation = False
    m.main()


if __name__ == '__main__':
    conftest.run_this_test(__file__)
