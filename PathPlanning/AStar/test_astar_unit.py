import unittest
from PathPlanning.AStar.a_star import AStarPlanner


class TestAStar(unittest.TestCase):
    def test_basic_path(self):
        # create simple U-shaped obstacle walls
        ox = [i for i in range(60)]          # bottom wall
        oy = [0 for _ in range(60)]

        for i in range(60):                  # right wall
            ox.append(60)
            oy.append(i)

        sx = 10.0        # start (x, y)
        sy = 10.0
        gx = 50.0        # goal (x, y)
        gy = 50.0
        grid_size = 2.0
        robot_radius = 1.0

        planner = AStarPlanner(ox, oy, grid_size, robot_radius)
        rx, ry = planner.planning(sx, sy, gx, gy)

        # basic sanity checks
        self.assertTrue(len(rx) > 0)
        self.assertEqual(len(rx), len(ry))

        # the algorithm may return the path in either direction;
        # verify the two endpoints match {start, goal} regardless of order
        self.assertCountEqual(
            {(rx[0], ry[0]), (rx[-1], ry[-1])},
            {(sx, sy), (gx, gy)}
        )

    def test_start_equals_goal(self):
        # provide ONE dummy obstacle so min()/max() calls succeed
        sx = sy = gx = gy = 10.0
        ox = [sx]
        oy = [sy]
        grid_size = 1.0
        robot_radius = 1.0

        planner = AStarPlanner(ox, oy, grid_size, robot_radius)
        rx, ry = planner.planning(sx, sy, gx, gy)

        # when start == goal, path should contain exactly that single point
        self.assertEqual(len(rx), 1)
        self.assertEqual((rx[0], ry[0]), (sx, sy))


if __name__ == '__main__':
    unittest.main()
