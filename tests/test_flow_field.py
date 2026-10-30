import conftest
import numpy as np
import PathPlanning.FlowField.flowfield as flow_field


def test():
    flow_field.show_animation = False
    flow_field.main()


def test_free_grid_with_goal_at_boundary():
    flow_field.show_animation = False
    grid = {(x, y): 'free' for x in range(3) for y in range(3)}
    planner = flow_field.FlowField(grid, 2, 2, 0, 0, 3, 3)
    planner.find_path()

    assert planner.integration_field[(2, 2)] == 0
    assert np.isfinite(planner.integration_field[(0, 0)])

    position = (0, 0)
    visited = {position}
    for _ in range(9):
        next_position = planner.vector_field[position]
        if next_position == (None, None):
            break
        assert next_position not in visited
        visited.add(next_position)
        position = next_position

    assert position == (2, 2)


def test_disconnected_start_has_no_navigation_direction():
    flow_field.show_animation = False
    grid = {
        (x, y): ('obs' if x == 1 else 'free')
        for x in range(4) for y in range(4)
    }
    planner = flow_field.FlowField(grid, 3, 3, 0, 0, 4, 4)
    planner.find_path()

    assert np.isinf(planner.integration_field[(0, 0)])
    assert planner.vector_field[(0, 0)] == (None, None)
    assert planner.integration_field[(3, 3)] == 0


def test_single_cell_grid_is_valid():
    flow_field.show_animation = False
    planner = flow_field.FlowField({(0, 0): 'free'}, 0, 0, 0, 0, 1, 1)
    planner.find_path()

    assert planner.integration_field[(0, 0)] == 0
    assert planner.vector_field[(0, 0)] == (None, None)




def test_animation_skips_termination_sentinel(monkeypatch):
    """Do not send (None, None) to Matplotlib after reaching the goal."""
    monkeypatch.setattr(flow_field, 'show_animation', True)
    plotted = []
    shown = []
    monkeypatch.setattr(
        flow_field.plt, 'plot',
        lambda x, y, *args: plotted.append((x, y)),
    )
    monkeypatch.setattr(flow_field.plt, 'pause', lambda *_: None)
    monkeypatch.setattr(flow_field.plt, 'show', lambda: shown.append(True))

    grid = {(0, 0): 'free', (1, 0): 'free'}
    planner = flow_field.FlowField(grid, 1, 0, 0, 0, 2, 1)
    planner.find_path()

    assert plotted == [(1, 0)]
    assert shown == [True]


if __name__ == '__main__':
    conftest.run_this_test(__file__)
