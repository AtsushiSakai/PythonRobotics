import conftest
from AerialNavigation.drone_3d_trajectory_following \
    import drone_3d_trajectory_following as m


def test1():
    m.show_animation = False
    m.main()


def test_poses_can_be_drawn(monkeypatch):
    # With show_animation, Quadrotor.plot() draws every pose with
    # transformation_matrix(), which needs scalar angles and positions.
    class QuadrotorWithoutDrawing(m.Quadrotor):
        def update_pose(self, *pose):
            super().update_pose(*pose)
            self.transformation_matrix()

    monkeypatch.setattr(m, "Quadrotor", QuadrotorWithoutDrawing)
    monkeypatch.setattr(m, "show_animation", False)
    m.main()


if __name__ == '__main__':
    conftest.run_this_test(__file__)
