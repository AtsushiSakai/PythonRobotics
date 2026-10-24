import conftest
from PathPlanning.DStarLite import d_star_lite as m


def test_1():
    """Test D* Lite with default configuration"""
    m.show_animation = False
    m.main()


def test_path_found():
    """Test that D* Lite successfully finds a path"""
    m.show_animation = False

    # Start and goal position
    sx = 10
    sy = 10
    gx = 50
    gy = 50

    ox, oy = [], []
    for i in range(-10, 60):
        ox.append(i)
        oy.append(-10.0)
    for i in range(-10, 60):
        ox.append(60.0)
        oy.append(i)
    for i in range(-10, 61):
        ox.append(i)
        oy.append(60.0)
    for i in range(-10, 61):
        ox.append(-10.0)
        oy.append(i)
    for i in range(-10, 40):
        ox.append(20.0)
        oy.append(i)
    for i in range(0, 40):
        ox.append(40.0)
        oy.append(60.0 - i)

    spoofed_ox = [[], [], [],
                  [i for i in range(0, 21)] + [0 for _ in range(0, 20)]]
    spoofed_oy = [[], [], [],
                  [20 for _ in range(0, 21)] + [i for i in range(0, 20)]]

    dstarlite = m.DStarLite(ox, oy)
    path_found, pathx, pathy = dstarlite.main(
        m.Node(x=sx, y=sy),
        m.Node(x=gx, y=gy),
        spoofed_ox=spoofed_ox,
        spoofed_oy=spoofed_oy
    )

    assert path_found, "D* Lite should find a path"
    assert len(pathx) > 0, "Path should contain points"
    assert len(pathy) > 0, "Path should contain points"
    assert pathx[0] == sx, "Path should start at start position"
    assert pathy[0] == sy, "Path should start at start position"
    assert pathx[-1] == gx, "Path should end at goal position"
    assert pathy[-1] == gy, "Path should end at goal position"


if __name__ == '__main__':
    conftest.run_this_test(__file__)
