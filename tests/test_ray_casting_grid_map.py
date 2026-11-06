import conftest  # Add root path to sys.path
from Mapping.ray_casting_grid_map import ray_casting_grid_map as m


def test1():
    m.show_animation = False
    m.main()


def test_observed_cells_survive_overlapping_shadows():
    def cast(ranges):
        grid, minx, _, miny, _, _ = m.generate_ray_casting_grid_map(
            ranges, [0.0, 0.0], 1.0, 0.1)
        return [grid[x - minx][-miny] for x in range(1, 6)]

    assert cast([2, 4]) == cast([4, 2]) == [0.0, 1.0, 0.5, 1.0, 0.5]


if __name__ == '__main__':
    conftest.run_this_test(__file__)
