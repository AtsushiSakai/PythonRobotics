import conftest  # Add root path to sys.path
from Mapping.point_cloud_sampling import point_cloud_sampling as m


def test_1(capsys):
    m.do_plot = False
    m.main()
    captured = capsys.readouterr()
    assert "voxel_sampling_points.shape=(27, 3)" in captured.out


if __name__ == '__main__':
    conftest.run_this_test(__file__)
