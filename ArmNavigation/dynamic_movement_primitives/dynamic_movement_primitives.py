"""
Author: Jonathan Schwartz (github.com/SchwartzCode)
More information on Dynamic Movement Primitives available at:
https://arxiv.org/abs/2102.03861
https://www.frontiersin.org/articles/10.3389/fncom.2013.00138/full
"""
import matplotlib.pyplot as plt
import numpy as np
import copy
import math

class ObstacleCircle(object):
    def __init__(self, origin, radius):
        self.x = origin[0]
        self.y = origin[1]
        self.radius = radius

    def distance_to(self, item):
        if isinstance(item, ObstacleCircle):
            return math.sqrt((self.x - item.x)**2 + (self.y - item.y)**2)
        else:
            # assume point that can be indexed
            return math.sqrt((self.x - item[0])**2 + (self.y - item[1])**2)

    def __repr__(self):
        return "Origin: (" + str(self.x) + ", " + str(self.y) + ") ~ r" + str(self.radius)

class CircleEvent(ObstacleCircle):

    def __init__(self, circle, start_pt_idx):
        self.circle = circle
        self.start_pt_idx = start_pt_idx
        self.end_pt_idx = None

    def __repr__(self):
        return "Origin: (" + str(self.x) + ", " + str(self.y) + ") ~ r" + str(self.radius)

class DMP(object):

    def __init__(self, training_data, data_period, K=156.25, B=25,
                 timesteps=500, repel_factor=0.001):
        """
        Arguments:
            training_data - data in for [(x1,y1), (x2,y2), ...]
            data_period   - amount of time training data covers
            K and B       - spring and damper constants to define DMP behavior
            timesteps     - number of points in generated trajectories
            repel_factor  - controls how much path will avoid obstacles
        """
        self.K = K  # virtual spring constant
        self.B = B  # virtual damper coefficient

        self.dt = data_period / timesteps
        self.timesteps = timesteps

        self.weights = None  # weights used to generate DMP trajectories
        self.obstacles = None

        self.repel_factor = repel_factor
        self.avoidance_distance = 0.1
        self.DMP_path_attraction = 10

        # self.T_orig = data_period
        # self.training_data = training_data

        self.find_basis_functions_weights(training_data, data_period)

    def find_basis_functions_weights(self, training_data, data_period, num_weights=10):
        """
        Arguments:
            data [(steps x spacial dim) np array] - data to replicate with DMP
            data_period [float] - time duration of data
        """

        self.training_data = training_data # for plotting

        dt = data_period / len(training_data)

        init_state = training_data[0]  # initial pos
        goal_state = training_data[-1]  # assume goal is reached by end of data

        # means (C) and std devs (H) of gaussian basis functions
        C = np.linspace(0, 1, num_weights)
        H = (0.65*(1./(num_weights-1))**2)

        for dim in range(len(training_data[0])):

            dimension_data = training_data[:,dim]

            q0 = init_state[dim]
            g = goal_state[dim]

            q = q0
            qd_last = 0

            phi_vals = []
            f_vals = []

            for i, _ in enumerate(dimension_data):
                if i + 1 == len(dimension_data):
                    qd = 0
                else:
                    qd = (dimension_data[i+1] - dimension_data[i]) / dt

                Phi = [np.exp(-0.5 * ((i * dt / data_period) - c)**2 / H)
                       for c in C]
                Phi = Phi/np.sum(Phi)

                qdd = (qd - qd_last)/dt

                f = (qdd * data_period**2 - self.K * (g - q) + self.B * qd
                     * data_period) / (g - q0)

                phi_vals.append(Phi)
                f_vals.append(f)

                qd_last = qd
                q += qd * dt

            phi_vals = np.asarray(phi_vals)
            f_vals = np.asarray(f_vals)

            w = np.linalg.lstsq(phi_vals, f_vals, rcond=None)

            if self.weights is None:
                self.weights = np.asarray(w[0])
            else:
                self.weights = np.vstack([self.weights, w[0]])

        return self.weights  # TODO: neccesary?

    def recreate_trajectory(self, init_state, goal_state, T, path):
        """
        init_state - initial state/position
        goal_state - goal state/position
        T  - amount of time to travek q0 -> g
        path - TODO
        """

        nrBasis = len(self.weights[0])  # number of gaussian basis functions

        # means (C) and std devs (H) of gaussian basis functions
        C = np.linspace(0, 1, nrBasis)
        H = (0.65*(1./(nrBasis-1))**2)

        # initialize virtual system
        t = 0

        # for plotting
        self.train_t_vals = np.arange(0, T, self.timesteps)
        # TODO: is self.period variable used

        if not isinstance(init_state, np.ndarray):
            init_state = np.asarray(init_state)
        if not isinstance(goal_state, np.ndarray):
            goal_state = np.asarray(goal_state)

        q = init_state
        dimensions = self.weights.shape[0]
        qd = np.zeros(dimensions)
        qdd = np.zeros(dimensions)

        positions = []
        for k in range(self.timesteps):
            new_state = []
            t = t + self.dt

            if path is not None:

                self.account_for_obstacles(path)

                obs_force = np.zeros(dimensions)
                path_norm_vec = np.zeros(2)
                if k+1 < self.timesteps:
                    path_norm_vec = self.get_vec_normal_to_path(path[k], path[k+1])

                # print("path norm vec q", path_norm_vec, q)
                if self.obstacles is not None:
                    obs_force = self.get_obstacle_force((q+path[k])/2, path_norm_vec)
                goal_dist = self.dist_between(q, goal_state)

                if 0.02 > goal_dist:
                    break

                goal_force = 0

                qdd = self.B*qd/T
                # - obs_force
                q = path[k] - obs_force

            else:

                qdd = np.zeros(dimensions)

                for dim in range(dimensions):

                    if t <= T:
                        Phi = [np.exp(-0.5 * ((t / T) - c)**2 / H) for c in C]
                        Phi = Phi / np.sum(Phi)
                        f = np.dot(Phi, self.weights[dim])
                    else:
                        f = 0

                    # simulate dynamics
                    qdd[dim] = self.K*(goal_state[dim] - q[dim])/T**2 - self.B*qd[dim]/T + (goal_state[dim] - init_state[dim])*f/T**2

            qd = qd + qdd * self.dt
            q = q + qd * self.dt

            # TODO: get rid of this
            if not isinstance(q, list):
                new_state = q.tolist()

            positions.append(new_state)

        pos_arr = np.asarray(positions)
        dists = []
        for i in range(1,pos_arr.shape[0]):
            dists.append(self.dist_between(pos_arr[i-1], pos_arr[i]))
        dists.sort()

        return np.arange(0, self.timesteps * self.dt, self.dt), positions

    def get_vec_normal_to_path(self, last_state, next_state):
        diffs = np.array([next_state[0] - last_state[0], next_state[1] - last_state[1]])
        normal_vec = np.array([diffs[0], -1 / diffs[1]])
        return normal_vec / np.linalg.norm(normal_vec)

    def account_for_obstacles(self, path):
        obstacle_circles = []
        for obs in self.obstacles:
            obstacle_circles.append(ObstacleCircle(obs, self.avoidance_distance))

        # combine circles that overlap
        i = 0
        while(i < len(obstacle_circles)-1):
            j = i+1
            while(j < len(obstacle_circles)):
                circ_dist = obstacle_circles[i].distance_to(obstacle_circles[j])
                if circ_dist < max(obstacle_circles[i].radius, obstacle_circles[j].radius):

                    circ2 = obstacle_circles.pop(j)
                    circ1 = obstacle_circles.pop(i)
                    new_circ_x = (circ1.x + circ2.x)/2
                    new_circ_y = (circ1.y + circ2.y)/2
                    new_circ_rad = max(circ1.radius, circ2.radius) + circ_dist

                    obstacle_circles.append(ObstacleCircle([new_circ_x, new_circ_y], new_circ_rad))
                    i = -1  # gets set to 0 by i++ below
                    break

                else:
                    j += 1
            i += 1

        circle_events = self.get_circle_events(path, obstacle_circles)

        for event in circle_events:
            start_pt = path[event.start_pt_idx]
            end_pt = path[event.end_pt_idx]

            circ_center = event.circle

            start_ang = math.atan2(start_pt[1] - circ_center[1],
                                   start_pt[0] - circ_center[0])

            end_ang = math.atan2(end_pt[1] - circ_center[1],
                                 end_pt[0] - circ_center[0])

            count_up = False

            if(start_ang > end_ang):
                if(start_ang - end_ang < math.pi):
                    count_up = True
            else:
                if(end_ang - start_ang < math.pi):
                    count_up = True

            # just find point that is halfway through arc and use that

        """
        TODO

        - draw perpendicular line to path through circle center
        - of 2 intersections perp line makes with circ, use one that is closer to path
        - draw dubins from before first int to that point
        - draw dubins from that point to after second int
        """

    def get_circle_events(self, path, circles):

        circle_flags = np.zeros(len(circles))

        circle_events = []
        for i in range(len(circles)):
            circle_events.append(None)

        for path_dex, path_pt in enumerate(path):
            for i, circ in enumerate(circles):
                dist = circ.distance_to(path_pt)
                if dist < self.avoidance_distance and not circle_flags[i]:
                    # circle in range
                    circle_flags[i] = 1
                    circle_events[i] = (CircleEvent(circ, path_dex))
                elif dist > self.avoidance_distance and circle_flags[i]:
                    # circle out of range
                    circle_flags[i] = 0
                    circle_events[i].end_pt_idx = path_dex

        return circle_events

    def get_obstacle_force(self, state, normal_vec):

        obstacle_force = np.zeros(len(self.obstacles[0]))

        for obs in self.obstacles:
            new_force = []

            dist = np.sum(np.sqrt((state - obs)**2))


            force = self.repel_factor / dist**2

            # print("\t obs dist force the quantity", obs, dist, force, ((obs - state) @ normal_vec))


            obstacle_force += force * ((obs - state) @ normal_vec) / np.linalg.norm(obs - state)
            # TODO: all lists or all np arrays for inputs
            # for dim in range(len(self.obstacles[0])):
            #     obstacle_force[dim] = (force * (obs[dim] - state[dim]))

        return obstacle_force

    def goal_attraction(self, state, path_point):

        state = np.asarray(state)

        dx = state[0] - path_point[0]
        dy = state[1] - path_point[1]
        dist = -np.sqrt(dx**2 + dy**2)

        att_force = np.array([dx/dist, dy/dist])*self.DMP_path_attraction
        return att_force

    @staticmethod
    def dist_between(p1, p2):
        return np.linalg.norm(p1 - p2)

    def solve_trajectory(self, q0, g, T, visualize=True, obstacles=None):

        _, path = self.recreate_trajectory(q0, g, T, None)

        self.obstacles = obstacles
        t, pos = self.recreate_trajectory(q0, g, T, path)
        state_hist = np.asarray(pos)
        path = np.asarray(path)

        # state_hist = np.asarray(pos)

        if visualize:
            if self.training_data.shape[1] == 2:
                if self.obstacles is not None:
                    for i, obs in enumerate(self.obstacles):
                        if i == 0:
                            plt.scatter(obs[0], obs[1], color='k', label='Obstacle')
                        else:
                            plt.scatter(obs[0], obs[1], color='k')
                plt.plot(self.training_data[:,0], self.training_data[:,1],
                         label="Training Data")
                plt.plot(path[:,0], path[:,1],
                         label="DMP Approximation")
                plt.plot(state_hist[:,0], state_hist[:,1],
                         label="path")

                plt.xlabel("X Position")
                plt.ylabel("Y Position")
                plt.legend()
                plt.show()

        return t, pos

    def show_DMP_purpose(self):
        """
        This function conveys the purpose of DMPs:
            to capture a trajectory and be able to stretch
            and squeeze it in terms of start and stop position
            or time
        """
        q0_orig = self.training_data[0]
        g_orig = self.training_data[-1]

        t_norm, pos_norm = self.recreate_trajectory(q0_orig,
                                                    g_orig,
                                                    self.T_orig)
        t_fast, pos_fast = self.recreate_trajectory(q0_orig,
                                                    g_orig,
                                                    self.T_orig/2)
        t_close, pos_close = self.recreate_trajectory(q0_orig,
                                                      g_orig/2,
                                                      self.T_orig)

        plt.plot(self.train_t_vals, self.training_data,
                 label="Training Data")
        plt.plot(t_norm, pos_norm, label="DMP Approximation",
                 linestyle='--')
        plt.plot(t_fast, pos_fast, label="Decreasing time duration",
                 linestyle='--')
        plt.plot(t_close, pos_close, label='Decreasing goal position',
                 linestyle='--')
        plt.xlabel("X Position")
        plt.ylabel("Y Position")
        plt.legend()
        plt.show()


if __name__ == '__main__':
    period = 2*np.pi
    # t = np.arange(0, np.pi/2, 0.01)
    # training_data = np.asarray([np.sin(t) + 0.02*np.random.rand(t.shape[0]),
    #                             np.cos(t) + 0.02*np.random.rand(t.shape[0]) ]).T
    t = np.arange(0,1,0.01)
    training_data = np.asarray([t, np.flip(t,0)]).T

    DMP_controller = DMP(training_data, period)
    # DMP_controller.show_DMP_purpose()

    obs = np.asarray([[0.3, 0.6], [0.7,0.25]])
    # obs = np.asarray([[0.435,0.9], [0.87,0.5]])
    # obs = np.asarray([[0.435,0.8]])
    # obs = np.asarray([[0.7,0.7]])
    DMP_controller.solve_trajectory([0,1], [1,0], 3, obstacles=obs)

"""
okay here's how to handle the obstacles:
- draw circle around each obstacle point
- dubins curve from two points before first intersection to furthest point on circle
    (create angle with obstacle point as center & the two points [next to intersection points && not inside circle]
    and draw bisector of circle_radius, add point with heading tangent to circ there)
      -> probably want points to be slightly further than radius length to give dubins some slack to work with
- dubins curve from the two circle tangent points from bisector to the two points right after second intersection
- to handle close obstacles:
    for multiple obs in obstacles:
        if dist_nearest_obstacle(obs) < turning_radius:
            -average two points
            - draw larger circle around average (~1.5x max(turning_radius, dist(obs, nearest_obs)))
            - remove two points from obs, add larger circ (or maybe just have a separate storage for these)
              - next point is skipped in loop
    - keep doing above loop until it loops through all points without making a change



"""
