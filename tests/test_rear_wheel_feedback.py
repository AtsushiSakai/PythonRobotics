import conftest  # Add root path to sys.path
from PathTracking.rear_wheel_feedback import rear_wheel_feedback as m


def test1(self):
    m.show_animation = False
    m.main()
