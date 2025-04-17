import conftest
from PathPlanning.Catmull_RomSplinePath.catmull_rom_spline_path import catmull_rom_spline

def test_catmull_rom_spline():
    way_points = [[0, 0], [1, 2], [2, 0], [3, 3]]
    num_points = 100

    spline_x, spline_y = catmull_rom_spline(way_points, num_points)
    
    assert spline_x.size > 0, "Spline X coordinates should not be empty"
    assert spline_y.size > 0, "Spline Y coordinates should not be empty"
    
    assert spline_x.shape == spline_y.shape, "Spline X and Y coordinates should have the same shape"

if __name__ == '__main__':
    conftest.run_this_test(__file__)
