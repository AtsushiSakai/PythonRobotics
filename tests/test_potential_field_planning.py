import conftest  # Add root path to sys.path
from PathPlanning.PotentialFieldPlanning import potential_field_planning as m


def test1():
    m.show_animation = False
    m.main()
