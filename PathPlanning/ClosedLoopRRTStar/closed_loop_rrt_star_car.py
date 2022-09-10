"""

Path planning Sample Code with Closed loop RRT for car like robot.

author: AtsushiSakai(@Atsushi_twi)

"""
import matplotlib.pyplot as plt
import numpy as np

import sys
import pathlib
sys.path.append(str(pathlib.Path(__file__).parent.parent))

from ClosedLoopRRTStar import pure_pursuit
from ClosedLoopRRTStar import unicycle_model
from ReedsSheppPath import reeds_shepp_path_planning
from RRTStarReedsShepp.rrt_star_reeds_shepp import RRTStarReedsShepp

show_animation = True


class ClosedLoopRRTStar(RRTStarReedsShepp):
    """
    Class for Closed loop RRT star planning
    """

    def __init__(self, start, goal, obstacle_list, rand_area,
                 max_iter=200,
                 connect_circle_dist=50.0,
                 robot_radius=0.0
                 ):
        super().__init__(start, goal, obstacle_list, rand_area,
                         max_iter=max_iter,
                         connect_circle_dist=connect_circle_dist,
                         robot_radius=robot_radius
                         )

        self.target_speed = 10.0 / 3.6
        self.yaw_th = np.deg2rad(3.0)
        self.xy_th = 0.5
        self.invalid_travel_ratio = 5.0

    def planning(self, animation=True):
        """
        do planning

        animation: flag for animation on or off
        """
        # planning with RRTStarReedsShepp
        super().planning(animation=animation)

        # generate coruse
        path_indexs = self.get_goal_indexes()

        flag, x, y, yaw, v, t, a, d = self.search_best_feasible_path(
            path_indexs)

        return flag, x, y, yaw, v, t, a, d

    def search_best_feasible_path(self, path_indexs):

        print("Start search feasible path")

        best_time = float("inf")

        fx, fy, fyaw, fv, ft, fa, fd = None, None, None, None, None, None, None

        # pure pursuit tracking
        for ind in path_indexs:
            path = self.generate_final_course(ind)

            flag, x, y, yaw, v, t, a, d = self.check_tracking_path_is_feasible(
                path)

            if flag and best_time >= t[-1]:
                print("feasible path is found")
                best_time = t[-1]
                fx, fy, fyaw, fv, ft, fa, fd = x, y, yaw, v, t, a, d

        print("best time is")
        print(best_time)

        if fx:
            fx.append(self.end.x)
            fy.append(self.end.y)
            fyaw.append(self.end.yaw)
            return True, fx, fy, fyaw, fv, ft, fa, fd

        return False, None, None, None, None, None, None, None

    def check_tracking_path_is_feasible(self, path):
        cx = np.array([state[0] for state in path])[::-1]
        cy = np.array([state[1] for state in path])[::-1]
        cyaw = np.array([state[2] for state in path])[::-1]

        goal = [cx[-1], cy[-1], cyaw[-1]]

        cx, cy, cyaw = pure_pursuit.extend_path(cx, cy, cyaw)

        speed_profile = pure_pursuit.calc_speed_profile(
            cx, cy, cyaw, self.target_speed)

        t, x, y, yaw, v, a, d, find_goal = pure_pursuit.closed_loop_prediction(
            cx, cy, cyaw, speed_profile, goal)
        yaw = [reeds_shepp_path_planning.pi_2_pi(iyaw) for iyaw in yaw]

        if not find_goal:
            print("cannot reach goal")

        if abs(yaw[-1] - goal[2]) >= self.yaw_th * 10.0:
            print("final angle is bad")
            find_goal = False

        travel = unicycle_model.dt * sum(np.abs(v))
        origin_travel = sum(np.hypot(np.diff(cx), np.diff(cy)))

        if (travel / origin_travel) >= self.invalid_travel_ratio:
            print("path is too long")
            find_goal = False

        tmp_node = self.Node(x, y, 0)
        tmp_node.path_x = x
        tmp_node.path_y = y
        if not self.check_collision(
                tmp_node, self.obstacle_list, self.robot_radius):
            print("This path is collision")
            find_goal = False

        return find_goal, x, y, yaw, v, t, a, d

    def get_goal_indexes(self):
        goalinds = []
        for (i, node) in enumerate(self.node_list):
            if self.calc_dist_to_goal(node.x, node.y) <= self.xy_th:
                goalinds.append(i)
        print("OK XY TH num is")
        print(len(goalinds))

        # angle check
        fgoalinds = []
        for i in goalinds:
            if abs(self.node_list[i].yaw - self.end.yaw) <= self.yaw_th:
                fgoalinds.append(i)
        print("OK YAW TH num is")
        print(len(fgoalinds))

        return fgoalinds


def main(gx=6.0, gy=7.0, gyaw=np.deg2rad(90.0), max_iter=100):
    print("Start" + __file__)
    # ====Search Path with RRT====
    obstacle_list = [
        (5, 5, 1),
        (4, 6, 1),
        (4, 8, 1),
        (4, 10, 1),
        (6, 5, 1),
        (7, 5, 1),
        (8, 6, 1),
        (8, 8, 1),
        (8, 10, 1)
    ]  # [x,y,size(radius)]

    # Set Initial parameters
    start = [0.0, 0.0, np.deg2rad(0.0)]
    goal = [gx, gy, gyaw]

    closed_loop_rrt_star = ClosedLoopRRTStar(start, goal,
                                             obstacle_list,
                                             [-2.0, 20.0],
                                             max_iter=max_iter)
    flag, x, y, yaw, v, t, a, d = closed_loop_rrt_star.planning(
        animation=show_animation)

    if not flag:
        print("cannot find feasible path")

    # Draw final path
    if show_animation:
        closed_loop_rrt_star.draw_graph()
        plt.plot(x, y, '-r')
        plt.grid(True)
        plt.pause(0.001)

        plt.subplots(1)
        plt.plot(t, [np.rad2deg(iyaw) for iyaw in yaw[:-1]], '-r')
        plt.xlabel("time[s]")
        plt.ylabel("Yaw[deg]")
        plt.grid(True)

        plt.subplots(1)
        plt.plot(t, [iv * 3.6 for iv in v], '-r')

        plt.xlabel("time[s]")
        plt.ylabel("velocity[km/h]")
        plt.grid(True)

        plt.subplots(1)
        plt.plot(t, a, '-r')
        plt.xlabel("time[s]")
        plt.ylabel("accel[m/ss]")
        plt.grid(True)

        plt.subplots(1)
        plt.plot(t, [np.rad2deg(td) for td in d], '-r')
        plt.xlabel("time[s]")
        plt.ylabel("Steering angle[deg]")
        plt.grid(True)

        plt.show()


if __name__ == '__main__':
    main()
