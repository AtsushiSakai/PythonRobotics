import conftest  # Add root path to sys.path
from PathTracking.pure_pursuit import pure_pursuit as m


def test1():
    m.show_animation = False
    m.main()

def test_backward():
    m.show_animation = False
    m.is_reverse_mode = True
    m.main()

if __name__ == '__main__':
    conftest.run_this_test(__file__)
