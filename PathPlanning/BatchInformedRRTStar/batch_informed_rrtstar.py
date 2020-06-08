"""
Batch Informed Trees based path planning:
Uses a heuristic to efficiently search increasingly dense
RGGs while reusing previous information. Provides faster
convergence that RRT*, Informed RRT* and other sampling based
methods.

Uses lazy connecting by combining sampling based methods and A*
like incremental graph search algorithms.

author: Karan Chawla(@karanchawla)
        Atsushi Sakai(@Atsushi_twi)

Reference: https://arxiv.org/abs/1405.5848
"""

import math
import random

import matplotlib.pyplot as plt
import numpy as np

show_animation = True


class RTree(object):
    # Class to represent the explicit tree created
    # while sampling through the state space

    def __init__(self, start=None, lowerLimit=None, upperLimit=None,
                 resolution=1.0):

        if upperLimit is None:
            upperLimit = [10, 10]
        if lowerLimit is None:
            lowerLimit = [0, 0]
        if start is None:
            start = [0, 0]
        self.vertices = dict()
        self.edges = []
        self.start = start
        self.lowerLimit = lowerLimit
        self.upperLimit = upperLimit
        self.dimension = len(lowerLimit)
        self.num_cells = [0] * self.dimension
        self.resolution = resolution
        # compute the number of grid cells based on the limits and
        # resolution given
        for idx in range(self.dimension):
            self.num_cells[idx] = np.ceil(
                (upperLimit[idx] - lowerLimit[idx]) / resolution)

        vertex_id = self.real_world_to_node_id(start)
        self.vertices[vertex_id] = []

    @staticmethod
    def get_root_id():
        # return the id of the root of the tree
        return 0

    def add_vertex(self, vertex):
        # add a vertex to the tree
        vertex_id = self.real_world_to_node_id(vertex)
        self.vertices[vertex_id] = []
        return vertex_id

    def add_edge(self, v, x):
        # create an edge between v and x vertices
        if (v, x) not in self.edges:
            self.edges.append((v, x))
        # since the tree is undirected
        self.vertices[v].append(x)
        self.vertices[x].append(v)

    def real_coords_to_grid_coord(self, real_coord):
        # convert real world coordinates to grid space
        # depends on the resolution of the grid
        # the output is the same as real world coords if the resolution
        # is set to 1
        coord = [0] * self.dimension
        for i in range(len(coord)):
            start = self.lowerLimit[i]  # start of the grid space
            coord[i] = int(np.around((real_coord[i] - start) / self.resolution))
        return coord

    def grid_coordinate_to_node_id(self, coord):
        # This function maps a grid coordinate to a unique
        # node id
        nodeId = 0
        for i in range(len(coord) - 1, -1, -1):
            product = 1
            for j in range(0, i):
                product = product * self.num_cells[j]
            nodeId = nodeId + coord[i] * product
        return nodeId

    def real_world_to_node_id(self, real_coord):
        # first convert the given coordinates to grid space and then
        # convert the grid space coordinates to a unique node id
        return self.grid_coordinate_to_node_id(
            self.real_coords_to_grid_coord(real_coord))

    def grid_coord_to_real_world_coord(self, coord):
        # This function maps a grid coordinate in discrete space
        # to a configuration in the full configuration space
        config = [0] * self.dimension
        for i in range(0, len(coord)):
            # start of the real world / configuration space
            start = self.lowerLimit[i]
            # step from the coordinate in the grid
            grid_step = self.resolution * coord[i]
            config[i] = start + grid_step
        return config

    def node_id_to_grid_coord(self, node_id):
        # This function maps a node id to the associated
        # grid coordinate
        coord = [0] * len(self.lowerLimit)
        for i in range(len(coord) - 1, -1, -1):
            # Get the product of the grid space maximums
            prod = 1
            for j in range(0, i):
                prod = prod * self.num_cells[j]
            coord[i] = np.floor(node_id / prod)
            node_id = node_id - (coord[i] * prod)
        return coord

    def node_id_to_real_world_coord(self, nid):
        # This function maps a node in discrete space to a configuration
        # in the full configuration space
        return self.grid_coord_to_real_world_coord(
            self.node_id_to_grid_coord(nid))


class BITStar(object):

    def __init__(self, start, goal,
                 obstacleList, randArea, eta=2.0,
                 maxIter=80):
        self.start = start
        self.goal = goal

        self.min_rand = randArea[0]
        self.max_rand = randArea[1]
        self.max_iIter = maxIter
        self.obstacleList = obstacleList
        self.startId = None
        self.goalId = None

        self.vertex_queue = []
        self.edge_queue = []
        self.samples = dict()
        self.g_scores = dict()
        self.f_scores = dict()
        self.nodes = dict()
        self.r = float('inf')
        self.eta = eta  # tunable parameter
        self.unit_ball_measure = 1
        self.old_vertices = []

        # initialize tree
        lowerLimit = [randArea[0], randArea[0]]
        upperLimit = [randArea[1], randArea[1]]
        self.tree = RTree(start=start, lowerLimit=lowerLimit,
                          upperLimit=upperLimit, resolution=0.01)

    def setup_planning(self):
        self.startId = self.tree.real_world_to_node_id(self.start)
        self.goalId = self.tree.real_world_to_node_id(self.goal)

        # add goal to the samples
        self.samples[self.goalId] = self.goal
        self.g_scores[self.goalId] = float('inf')
        self.f_scores[self.goalId] = 0

        # add the start id to the tree
        self.tree.add_vertex(self.start)
        self.g_scores[self.startId] = 0
        self.f_scores[self.startId] = self.compute_heuristic_cost(
            self.startId, self.goalId)

        # max length we expect to find in our 'informed' sample space, starts as infinite
        cBest = self.g_scores[self.goalId]

        # Computing the sampling space
        cMin = math.hypot(self.start[0] - self.goal[0],
                          self.start[1] - self.goal[1]) / 1.5
        xCenter = np.array([[(self.start[0] + self.goal[0]) / 2.0],
                            [(self.start[1] + self.goal[1]) / 2.0], [0]])
        a1 = np.array([[(self.goal[0] - self.start[0]) / cMin],
                       [(self.goal[1] - self.start[1]) / cMin], [0]])
        eTheta = math.atan2(a1[1], a1[0])
        # first column of identity matrix transposed
        id1_t = np.array([1.0, 0.0, 0.0]).reshape(1, 3)
        M = np.dot(a1, id1_t)
        U, S, Vh = np.linalg.svd(M, True, True)
        C = np.dot(np.dot(U, np.diag(
            [1.0, 1.0, np.linalg.det(U) * np.linalg.det(np.transpose(Vh))])),
                   Vh)

        self.samples.update(self.informed_sample(
            200, cBest, cMin, xCenter, C))

        return eTheta, cMin, xCenter, C, cBest

    def setup_sample(self, iterations, foundGoal, cMin, xCenter, C, cBest):

        if len(self.vertex_queue) == 0 and len(self.edge_queue) == 0:
            print("Batch: ", iterations)
            # Using informed rrt star way of computing the samples
            self.r = 2.0
            if iterations != 0:
                if foundGoal:
                    # a better way to do this would be to make number of samples
                    # a function of cMin
                    m = 200
                    self.samples = dict()
                    self.samples[self.goalId] = self.goal
                else:
                    m = 100
                cBest = self.g_scores[self.goalId]
                self.samples.update(self.informed_sample(
                    m, cBest, cMin, xCenter, C))

            # make the old vertices the new vertices
            self.old_vertices += self.tree.vertices.keys()
            # add the vertices to the vertex queue
            for nid in self.tree.vertices.keys():
                if nid not in self.vertex_queue:
                    self.vertex_queue.append(nid)
        return cBest

    def plan(self, animation=True):

        eTheta, cMin, xCenter, C, cBest = self.setup_planning()
        iterations = 0

        foundGoal = False
        # run until done
        while iterations < self.max_iIter:
            cBest = self.setup_sample(iterations,
                                      foundGoal, cMin, xCenter, C, cBest)
            # expand the best vertices until an edge is better than the vertex
            # this is done because the vertex cost represents the lower bound
            # on the edge cost
            while self.best_vertex_queue_value() <= \
                    self.best_edge_queue_value():
                self.expand_vertex(self.best_in_vertex_queue())

            # add the best edge to the tree
            bestEdge = self.best_in_edge_queue()
            self.edge_queue.remove(bestEdge)

            # Check if this can improve the current solution
            estimatedCostOfVertex = self.g_scores[bestEdge[
                0]] + self.compute_distance_cost(
                bestEdge[0], bestEdge[1]) + self.compute_heuristic_cost(
                bestEdge[1], self.goalId)
            estimatedCostOfEdge = self.compute_distance_cost(
                self.startId, bestEdge[0]) + self.compute_heuristic_cost(
                bestEdge[0], bestEdge[1]) + self.compute_heuristic_cost(
                bestEdge[1], self.goalId)
            actualCostOfEdge = self.g_scores[
                                   bestEdge[0]] + self.compute_distance_cost(
                bestEdge[0], bestEdge[1])

            f1 = estimatedCostOfVertex < self.g_scores[self.goalId]
            f2 = estimatedCostOfEdge < self.g_scores[self.goalId]
            f3 = actualCostOfEdge < self.g_scores[self.goalId]

            if f1 and f2 and f3:
                # connect this edge
                firstCoord = self.tree.node_id_to_real_world_coord(
                    bestEdge[0])
                secondCoord = self.tree.node_id_to_real_world_coord(
                    bestEdge[1])
                path = self.connect(firstCoord, secondCoord)
                lastEdge = self.tree.real_world_to_node_id(secondCoord)
                if path is None or len(path) == 0:
                    continue
                nextCoord = path[len(path) - 1, :]
                nextCoordPathId = self.tree.real_world_to_node_id(
                    nextCoord)
                bestEdge = (bestEdge[0], nextCoordPathId)
                if bestEdge[1] in self.tree.vertices.keys():
                    continue
                else:
                    try:
                        del self.samples[bestEdge[1]]
                    except KeyError:
                        # invalid sample key
                        pass
                    eid = self.tree.add_vertex(nextCoord)
                    self.vertex_queue.append(eid)
                if eid == self.goalId or bestEdge[0] == self.goalId or \
                        bestEdge[1] == self.goalId:
                    print("Goal found")
                    foundGoal = True

                self.tree.add_edge(bestEdge[0], bestEdge[1])

                g_score = self.compute_distance_cost(
                    bestEdge[0], bestEdge[1])
                self.g_scores[bestEdge[1]] = g_score + self.g_scores[
                    bestEdge[0]]
                self.f_scores[
                    bestEdge[1]] = g_score + self.compute_heuristic_cost(
                    bestEdge[1], self.goalId)
                self.update_graph()

                # visualize new edge
                if animation:
                    self.draw_graph(xCenter=xCenter, cBest=cBest,
                                    cMin=cMin, eTheta=eTheta,
                                    samples=self.samples.values(),
                                    start=firstCoord, end=secondCoord)

                self.remove_queue(lastEdge, bestEdge)

            else:
                print("Nothing good")
                self.edge_queue = []
                self.vertex_queue = []

            iterations += 1

        print("Finding the path")
        return self.find_final_path()

    def find_final_path(self):
        plan = [self.goal]
        currId = self.goalId
        while currId != self.startId:
            plan.append(self.tree.node_id_to_real_world_coord(currId))
            try:
                currId = self.nodes[currId]
            except KeyError:
                print("cannot find Path")
                return []

        plan.append(self.start)
        plan = plan[::-1]  # reverse the plan

        return plan

    def remove_queue(self, lastEdge, bestEdge):
        for edge in self.edge_queue:
            if edge[1] == bestEdge[1]:
                dist_cost = self.compute_distance_cost(edge[1], bestEdge[1])
                if self.g_scores[edge[1]] + dist_cost >= \
                        self.g_scores[self.goalId]:
                    if (lastEdge, bestEdge[1]) in self.edge_queue:
                        self.edge_queue.remove(
                            (lastEdge, bestEdge[1]))

    def connect(self, start, end):
        # A function which attempts to extend from a start coordinates
        # to goal coordinates
        steps = int(self.compute_distance_cost(
            self.tree.real_world_to_node_id(start),
            self.tree.real_world_to_node_id(end)) * 10)
        x = np.linspace(start[0], end[0], num=steps)
        y = np.linspace(start[1], end[1], num=steps)
        for i in range(len(x)):
            if self._collision_check(x[i], y[i]):
                if i == 0:
                    return None
                # if collision, send path until collision
                return np.vstack((x[0:i], y[0:i])).transpose()

        return np.vstack((x, y)).transpose()

    def _collision_check(self, x, y):
        for (ox, oy, size) in self.obstacleList:
            dx = ox - x
            dy = oy - y
            d = dx * dx + dy * dy
            if d <= size ** 2:
                return True  # collision
        return False

    def compute_heuristic_cost(self, start_id, goal_id):
        # Using Manhattan distance as heuristic
        start = np.array(self.tree.node_id_to_real_world_coord(start_id))
        goal = np.array(self.tree.node_id_to_real_world_coord(goal_id))

        return np.linalg.norm(start - goal, 2)

    def compute_distance_cost(self, vid, xid):
        # L2 norm distance
        start = np.array(self.tree.node_id_to_real_world_coord(vid))
        stop = np.array(self.tree.node_id_to_real_world_coord(xid))

        return np.linalg.norm(stop - start, 2)

    # Sample free space confined in the radius of ball R
    def informed_sample(self, m, cMax, cMin, xCenter, C):
        samples = dict()
        print("g_Score goal id: ", self.g_scores[self.goalId])
        for i in range(m + 1):
            if cMax < float('inf'):
                r = [cMax / 2.0,
                     math.sqrt(cMax ** 2 - cMin ** 2) / 2.0,
                     math.sqrt(cMax ** 2 - cMin ** 2) / 2.0]
                L = np.diag(r)
                xBall = self.sample_unit_ball()
                rnd = np.dot(np.dot(C, L), xBall) + xCenter
                rnd = [rnd[(0, 0)], rnd[(1, 0)]]
                random_id = self.tree.real_world_to_node_id(rnd)
                samples[random_id] = rnd
            else:
                rnd = self.sample_free_space()
                random_id = self.tree.real_world_to_node_id(rnd)
                samples[random_id] = rnd
        return samples

    # Sample point in a unit ball
    @staticmethod
    def sample_unit_ball():
        a = random.random()
        b = random.random()

        if b < a:
            a, b = b, a

        sample = (b * math.cos(2 * math.pi * a / b),
                  b * math.sin(2 * math.pi * a / b))
        return np.array([[sample[0]], [sample[1]], [0]])

    def sample_free_space(self):
        rnd = [random.uniform(self.min_rand, self.max_rand),
               random.uniform(self.min_rand, self.max_rand)]

        return rnd

    def best_vertex_queue_value(self):
        if len(self.vertex_queue) == 0:
            return float('inf')
        values = [self.g_scores[v]
                  + self.compute_heuristic_cost(v, self.goalId) for v in
                  self.vertex_queue]
        values.sort()
        return values[0]

    def best_edge_queue_value(self):
        if len(self.edge_queue) == 0:
            return float('inf')
        # return the best value in the queue by score g_tau[v] + c(v,x) + h(x)
        values = [self.g_scores[e[0]] + self.compute_distance_cost(e[0], e[1])
                  + self.compute_heuristic_cost(e[1], self.goalId) for e in
                  self.edge_queue]
        values.sort(reverse=True)
        return values[0]

    def best_in_vertex_queue(self):
        # return the best value in the vertex queue
        v_plus_values = [(v, self.g_scores[v] +
                          self.compute_heuristic_cost(v, self.goalId))
                         for v in self.vertex_queue]
        v_plus_values = sorted(v_plus_values, key=lambda x: x[1])
        # print(v_plus_values)
        return v_plus_values[0][0]

    def best_in_edge_queue(self):
        e_and_values = [
            (e[0], e[1], self.g_scores[e[0]] + self.compute_distance_cost(
                e[0], e[1]) + self.compute_heuristic_cost(e[1], self.goalId))
            for e in self.edge_queue]
        e_and_values = sorted(e_and_values, key=lambda x: x[2])

        return e_and_values[0][0], e_and_values[0][1]

    def expand_vertex(self, vid):
        self.vertex_queue.remove(vid)

        # get the coordinates for given vid
        currCoord = np.array(self.tree.node_id_to_real_world_coord(vid))

        # get the nearest value in vertex for every one in samples where difference is
        # less than the radius
        neighbors = []
        for sid, s_coord in self.samples.items():
            s_coord = np.array(s_coord)
            if np.linalg.norm(s_coord - currCoord, 2) <= self.r and sid != vid:
                neighbors.append((sid, s_coord))

        # add an edge to the edge queue is the path might improve the solution
        for neighbor in neighbors:
            sid = neighbor[0]
            h_cost = self.compute_heuristic_cost(sid, self.goalId)
            estimated_f_score = self.compute_distance_cost(
                self.startId, vid) + h_cost + self.compute_distance_cost(vid,
                                                                         sid)
            if estimated_f_score < self.g_scores[self.goalId]:
                self.edge_queue.append((vid, sid))

        # add the vertex to the edge queue
        self.add_vertex_to_edge_queue(vid, currCoord)

    def add_vertex_to_edge_queue(self, vid, currCoord):
        if vid not in self.old_vertices:
            neighbors = []
            for v, edges in self.tree.vertices.items():
                if v != vid and (v, vid) not in self.edge_queue and \
                        (vid, v) not in self.edge_queue:
                    v_coord = self.tree.node_id_to_real_world_coord(v)
                    if np.linalg.norm(currCoord - v_coord, 2) <= self.r:
                        neighbors.append((vid, v_coord))

            for neighbor in neighbors:
                sid = neighbor[0]
                estimated_f_score = self.compute_distance_cost(
                    self.startId, vid) + self.compute_distance_cost(
                    vid, sid) + self.compute_heuristic_cost(sid, self.goalId)
                if estimated_f_score < self.g_scores[self.goalId] and (
                        self.g_scores[vid] +
                        self.compute_distance_cost(vid, sid)) < \
                        self.g_scores[sid]:
                    self.edge_queue.append((vid, sid))

    def update_graph(self):
        closedSet = []
        openSet = []
        currId = self.startId
        openSet.append(currId)

        while len(openSet) != 0:
            # get the element with lowest f_score
            currId = min(openSet, key=lambda x: self.f_scores[x])

            # remove element from open set
            openSet.remove(currId)

            # Check if we're at the goal
            if currId == self.goalId:
                break

            if currId not in closedSet:
                closedSet.append(currId)

            # find a non visited successor to the current node
            successors = self.tree.vertices[currId]
            for successor in successors:
                if successor in closedSet:
                    continue
                else:
                    # calculate tentative g score
                    g_score = self.g_scores[currId] + \
                              self.compute_distance_cost(currId, successor)
                    if successor not in openSet:
                        # add the successor to open set
                        openSet.append(successor)
                    elif g_score >= self.g_scores[successor]:
                        continue

                    # update g and f scores
                    self.g_scores[successor] = g_score
                    self.f_scores[
                        successor] = g_score + self.compute_heuristic_cost(
                        successor, self.goalId)

                    # store the parent and child
                    self.nodes[successor] = currId

    def draw_graph(self, xCenter=None, cBest=None, cMin=None, eTheta=None,
                   samples=None, start=None, end=None):
        plt.clf()
        # for stopping simulation with the esc key.
        plt.gcf().canvas.mpl_connect(
            'key_release_event',
            lambda event: [exit(0) if event.key == 'escape' else None])
        for rnd in samples:
            if rnd is not None:
                plt.plot(rnd[0], rnd[1], "^k")
                if cBest != float('inf'):
                    self.plot_ellipse(xCenter, cBest, cMin, eTheta)

        if start is not None and end is not None:
            plt.plot([start[0], start[1]], [end[0], end[1]], "-g")

        for (ox, oy, size) in self.obstacleList:
            plt.plot(ox, oy, "ok", ms=30 * size)

        plt.plot(self.start[0], self.start[1], "xr")
        plt.plot(self.goal[0], self.goal[1], "xr")
        plt.axis([-2, 15, -2, 15])
        plt.grid(True)
        plt.pause(0.01)

    @staticmethod
    def plot_ellipse(xCenter, cBest, cMin, eTheta):  # pragma: no cover

        a = math.sqrt(cBest ** 2 - cMin ** 2) / 2.0
        b = cBest / 2.0
        angle = math.pi / 2.0 - eTheta
        cx = xCenter[0]
        cy = xCenter[1]

        t = np.arange(0, 2 * math.pi + 0.1, 0.1)
        x = [a * math.cos(it) for it in t]
        y = [b * math.sin(it) for it in t]
        R = np.array([[math.cos(angle), math.sin(angle)],
                      [-math.sin(angle), math.cos(angle)]])
        fx = R @ np.array([x, y])
        px = np.array(fx[0, :] + cx).flatten()
        py = np.array(fx[1, :] + cy).flatten()
        plt.plot(cx, cy, "xc")
        plt.plot(px, py, "--c")


def main(maxIter=80):
    print("Starting Batch Informed Trees Star planning")
    obstacleList = [
        (5, 5, 0.5),
        (9, 6, 1),
        (7, 5, 1),
        (1, 5, 1),
        (3, 6, 1),
        (7, 9, 1)
    ]

    bitStar = BITStar(start=[-1, 0], goal=[3, 8], obstacleList=obstacleList,
                      randArea=[-2, 15], maxIter=maxIter)
    path = bitStar.plan(animation=show_animation)
    print("Done")

    if show_animation:
        plt.plot([x for (x, y) in path], [y for (x, y) in path], '-r')
        plt.grid(True)
        plt.pause(0.05)
        plt.show()


if __name__ == '__main__':
    main()
