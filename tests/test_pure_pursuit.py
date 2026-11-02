import conftest  # Add root path to sys.path
from PathTracking.pure_pursuit import pure_pursuit as m
import pytest

def test1():
    m.show_animation = False
    m.main()

def test_backward():
    m.show_animation = False
    m.is_reverse_mode = True
    m.main()

@pytest.mark.parametrize(
    "course_x, rear_x_positions, expected_indices",
    [
        ([0.0], [0.0, 0.0], [0, 0]),
        ([0.0, 10.0, 20.0], [20.0, 20.0], [2, 2]),
        (
            [0.0, 10.0, 20.0],
            [0.0, 10.0, 20.0, 25.0],
            [1, 2, 2, 2],
        ),
        ([0.0, 10.0, 20.0], [0.0, 0.0], [1, 1]),
    ],
    ids=[
        "single-waypoint",
        "repeated-search-at-final-waypoint",
        "advance-to-and-beyond-final-waypoint",
        "repeated-search-at-interior-waypoint",
    ],
)
def test_search_target_index_handles_path_end(
    course_x, rear_x_positions, expected_indices
):
    course = m.TargetCourse(course_x, [0.0] * len(course_x))

    for rear_x, expected_index in zip(
        rear_x_positions, expected_indices
    ):
        state = m.State(
            x=rear_x + m.WB / 2,
            y=0.0,
            yaw=0.0,
            v=0.0,
        )

        target_index, _ = course.search_target_index(state)

        assert target_index == expected_index

if __name__ == '__main__':
    conftest.run_this_test(__file__)
