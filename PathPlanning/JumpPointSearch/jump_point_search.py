"""

Jump Point Search (JPS) grid planning

author: faysal-shah

Jump Point Search is an optimization of the A* search algorithm for uniform-cost
grid maps. It keeps the same optimal path cost as A* but expands far fewer nodes
by "jumping" over long sequences of symmetric, equivalent paths and only stopping
at so-called jump points (the start, the goal, and nodes that have a forced
neighbor). Because this implementation moves on the grid exactly like
``PathPlanning/AStar/a_star.py`` (eight-connected, diagonal moves cost
``sqrt(2)`` and are allowed as long as the target cell is free), it searches the
very same graph as that A* example and therefore returns a path with an identical
cost.

See the original paper:
    D. Harabor and A. Grastien,
    "Online Graph Pruning for Pathfinding on Grid Maps" (AAAI 2011)
    https://ojs.aaai.org/index.php/AAAI/article/view/7994

See also the Wikipedia article:
    https://en.wikipedia.org/wiki/Jump_point_search

"""

import math

import matplotlib.pyplot as plt

show_animation = True


class JumpPointSearchPlanner:

    def __init__(self, ox, oy, resolution, rr):
        """
        Initialize grid map for jump point search planning

        ox: x position list of Obstacles [m]
        oy: y position list of Obstacles [m]
        resolution: grid resolution [m]
        rr: robot radius[m]
        """

        self.resolution = resolution
        self.rr = rr
        self.min_x, self.min_y = 0, 0
        self.max_x, self.max_y = 0, 0
        self.obstacle_map = None
        self.x_width, self.y_width = 0, 0
        self.goal_x, self.goal_y = 0, 0
        self.motion = self.get_motion_model()
        self.calc_obstacle_map(ox, oy)

    class Node:
        def __init__(self, x, y, cost, parent_index,
                     direction_x=0, direction_y=0):
            self.x = x  # index of grid
            self.y = y  # index of grid
            self.cost = cost
            self.parent_index = parent_index
            # grid direction (dx, dy) travelled from the parent to this node.
            # (0, 0) marks the start node, which has no parent direction.
            self.direction_x = direction_x
            self.direction_y = direction_y

        def __str__(self):
            return str(self.x) + "," + str(self.y) + "," + str(
                self.cost) + "," + str(self.parent_index)

    def planning(self, sx, sy, gx, gy):
        """
        Jump point search path search

        input:
            s_x: start x position [m]
            s_y: start y position [m]
            gx: goal x position [m]
            gy: goal y position [m]

        output:
            rx: x position list of the final path
            ry: y position list of the final path
        """

        start_node = self.Node(self.calc_xy_index(sx, self.min_x),
                               self.calc_xy_index(sy, self.min_y), 0.0, -1)
        goal_node = self.Node(self.calc_xy_index(gx, self.min_x),
                              self.calc_xy_index(gy, self.min_y), 0.0, -1)
        self.goal_x, self.goal_y = goal_node.x, goal_node.y

        open_set, closed_set = dict(), dict()
        open_set[self.calc_grid_index(start_node)] = start_node

        while True:
            if len(open_set) == 0:
                print("Open set is empty..")
                break

            c_id = min(
                open_set,
                key=lambda o: open_set[o].cost + self.calc_heuristic(goal_node,
                                                                     open_set[
                                                                         o]))
            current = open_set[c_id]

            # show graph
            if show_animation:  # pragma: no cover
                plt.plot(self.calc_grid_position(current.x, self.min_x),
                         self.calc_grid_position(current.y, self.min_y), "xc")
                # for stopping simulation with the esc key.
                plt.gcf().canvas.mpl_connect('key_release_event',
                                             lambda event: [exit(
                                                 0) if event.key == 'escape' else None])
                if len(closed_set.keys()) % 10 == 0:
                    plt.pause(0.001)

            if current.x == goal_node.x and current.y == goal_node.y:
                print("Find goal")
                goal_node.parent_index = current.parent_index
                goal_node.cost = current.cost
                break

            # Remove the item from the open set
            del open_set[c_id]

            # Add it to the closed set
            closed_set[c_id] = current

            # expand search tree to the reachable jump points
            for direction_x, direction_y in self.find_neighbors(current):
                jump_point = self.jump(current.x, current.y,
                                       direction_x, direction_y)
                if jump_point is None:
                    continue

                jump_x, jump_y = jump_point
                node = self.Node(jump_x, jump_y,
                                 current.cost + self.move_cost(current.x,
                                                               current.y,
                                                               jump_x, jump_y),
                                 c_id, direction_x, direction_y)
                n_id = self.calc_grid_index(node)

                if n_id in closed_set:
                    continue

                if n_id not in open_set:
                    open_set[n_id] = node  # discovered a new jump point
                else:
                    if open_set[n_id].cost > node.cost:
                        # This path is the best until now. record it
                        open_set[n_id] = node

        rx, ry = self.calc_final_path(goal_node, closed_set)

        return rx, ry

    def find_neighbors(self, node):
        """
        Return the pruned set of travel directions (dx, dy) to search from a
        node. Only the natural neighbors and the forced neighbors (the pruning
        rules of Jump Point Search) are returned, which is what lets JPS skip
        the symmetric paths that plain A* would expand.
        """

        cx, cy = node.x, node.y
        dx, dy = node.direction_x, node.direction_y
        neighbors = []

        # The start node has no parent direction, so every direction is open.
        if dx == 0 and dy == 0:
            return list(self.motion)

        if dx != 0 and dy != 0:
            # Diagonal move: keep the diagonal itself and its two components.
            neighbors.append((dx, 0))
            neighbors.append((0, dy))
            neighbors.append((dx, dy))
            # Forced neighbors appear next to an obstacle that blocks the
            # alternative, equally short path around this node.
            if self.is_obstacle(cx - dx, cy) and self.is_free(cx - dx, cy + dy):
                neighbors.append((-dx, dy))
            if self.is_obstacle(cx, cy - dy) and self.is_free(cx + dx, cy - dy):
                neighbors.append((dx, -dy))
        elif dx != 0:
            # Horizontal move.
            neighbors.append((dx, 0))
            if self.is_obstacle(cx, cy + 1) and self.is_free(cx + dx, cy + 1):
                neighbors.append((dx, 1))
            if self.is_obstacle(cx, cy - 1) and self.is_free(cx + dx, cy - 1):
                neighbors.append((dx, -1))
        else:
            # Vertical move.
            neighbors.append((0, dy))
            if self.is_obstacle(cx + 1, cy) and self.is_free(cx + 1, cy + dy):
                neighbors.append((1, dy))
            if self.is_obstacle(cx - 1, cy) and self.is_free(cx - 1, cy + dy):
                neighbors.append((-1, dy))

        return neighbors

    def jump(self, cx, cy, dx, dy):
        """
        Recursively "jump" from (cx, cy) along direction (dx, dy) and return the
        next jump point, or None if the ray runs into an obstacle or leaves the
        map before reaching one.

        A cell is a jump point when it is the goal or it has a forced neighbor.
        A diagonal jump additionally stops whenever one of its two orthogonal
        component jumps finds a jump point.
        """

        nx, ny = cx + dx, cy + dy
        while True:
            if not self.is_free(nx, ny):
                return None

            if nx == self.goal_x and ny == self.goal_y:
                return nx, ny

            if dx != 0 and dy != 0:
                # Diagonal: stop on a forced neighbor around this cell.
                if (self.is_free(nx - dx, ny + dy)
                        and self.is_obstacle(nx - dx, ny)) \
                        or (self.is_free(nx + dx, ny - dy)
                            and self.is_obstacle(nx, ny - dy)):
                    return nx, ny
                # Stop if a horizontal or vertical jump finds a jump point.
                if self.jump(nx, ny, dx, 0) is not None \
                        or self.jump(nx, ny, 0, dy) is not None:
                    return nx, ny
            elif dx != 0:
                # Horizontal: forced neighbor above or below the next cell.
                if (self.is_free(nx + dx, ny + 1)
                        and self.is_obstacle(nx, ny + 1)) \
                        or (self.is_free(nx + dx, ny - 1)
                            and self.is_obstacle(nx, ny - 1)):
                    return nx, ny
            else:
                # Vertical: forced neighbor left or right of the next cell.
                if (self.is_free(nx + 1, ny + dy)
                        and self.is_obstacle(nx + 1, ny)) \
                        or (self.is_free(nx - 1, ny + dy)
                            and self.is_obstacle(nx - 1, ny)):
                    return nx, ny

            # Keep sliding in the same direction.
            nx += dx
            ny += dy

    @staticmethod
    def move_cost(from_x, from_y, to_x, to_y):
        """
        Cost of a single jump between two aligned cells. A jump always travels
        along one of the eight grid directions, so the two cells are either on a
        straight line or on a 45 degree diagonal.
        """

        delta_x = abs(to_x - from_x)
        delta_y = abs(to_y - from_y)
        if delta_x != 0 and delta_y != 0:
            return math.sqrt(2) * delta_x
        return delta_x + delta_y

    def calc_final_path(self, goal_node, closed_set):
        # Collect the jump points from the goal back to the start.
        jump_points = [(goal_node.x, goal_node.y)]
        parent_index = goal_node.parent_index
        while parent_index != -1:
            n = closed_set[parent_index]
            jump_points.append((n.x, n.y))
            parent_index = n.parent_index

        # Expand the jump points into the full cell-by-cell course so the output
        # matches the A* example format (goal first, start last).
        rx, ry = [], []
        for i in range(len(jump_points) - 1):
            from_x, from_y = jump_points[i]
            to_x, to_y = jump_points[i + 1]
            step_x = self.sign(to_x - from_x)
            step_y = self.sign(to_y - from_y)
            segment_length = max(abs(to_x - from_x), abs(to_y - from_y))
            for step in range(segment_length):
                ix = from_x + step_x * step
                iy = from_y + step_y * step
                rx.append(self.calc_grid_position(ix, self.min_x))
                ry.append(self.calc_grid_position(iy, self.min_y))

        last_x, last_y = jump_points[-1]
        rx.append(self.calc_grid_position(last_x, self.min_x))
        ry.append(self.calc_grid_position(last_y, self.min_y))

        return rx, ry

    @staticmethod
    def sign(value):
        if value > 0:
            return 1
        elif value < 0:
            return -1
        return 0

    @staticmethod
    def calc_heuristic(n1, n2):
        w = 1.0  # weight of heuristic
        d = w * math.hypot(n1.x - n2.x, n1.y - n2.y)
        return d

    def calc_grid_position(self, index, min_position):
        """
        calc grid position

        :param index:
        :param min_position:
        :return:
        """
        pos = index * self.resolution + min_position
        return pos

    def calc_xy_index(self, position, min_pos):
        return round((position - min_pos) / self.resolution)

    def calc_grid_index(self, node):
        return (node.y - self.min_y) * self.x_width + (node.x - self.min_x)

    def is_free(self, x, y):
        """Return True when cell (x, y) is inside the map and not an obstacle."""
        if x < 0 or x >= self.x_width:
            return False
        if y < 0 or y >= self.y_width:
            return False
        return not self.obstacle_map[x][y]

    def is_obstacle(self, x, y):
        """
        Return True when cell (x, y) blocks movement. Cells outside the map are
        treated as obstacles (walls), as in the A* example.
        """
        return not self.is_free(x, y)

    def calc_obstacle_map(self, ox, oy):

        self.min_x = round(min(ox))
        self.min_y = round(min(oy))
        self.max_x = round(max(ox))
        self.max_y = round(max(oy))
        print("min_x:", self.min_x)
        print("min_y:", self.min_y)
        print("max_x:", self.max_x)
        print("max_y:", self.max_y)

        self.x_width = round((self.max_x - self.min_x) / self.resolution)
        self.y_width = round((self.max_y - self.min_y) / self.resolution)
        print("x_width:", self.x_width)
        print("y_width:", self.y_width)

        # obstacle map generation
        self.obstacle_map = [[False for _ in range(self.y_width)]
                             for _ in range(self.x_width)]
        for ix in range(self.x_width):
            x = self.calc_grid_position(ix, self.min_x)
            for iy in range(self.y_width):
                y = self.calc_grid_position(iy, self.min_y)
                for iox, ioy in zip(ox, oy):
                    d = math.hypot(iox - x, ioy - y)
                    if d <= self.rr:
                        self.obstacle_map[ix][iy] = True
                        break

    @staticmethod
    def get_motion_model():
        # dx, dy (the eight grid travel directions)
        motion = [(1, 0),
                  (0, 1),
                  (-1, 0),
                  (0, -1),
                  (-1, -1),
                  (-1, 1),
                  (1, -1),
                  (1, 1)]

        return motion


def main():
    print(__file__ + " start!!")

    # start and goal position
    sx = 10.0  # [m]
    sy = 10.0  # [m]
    gx = 50.0  # [m]
    gy = 50.0  # [m]
    grid_size = 2.0  # [m]
    robot_radius = 1.0  # [m]

    # set obstacle positions
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

    if show_animation:  # pragma: no cover
        plt.plot(ox, oy, ".k")
        plt.plot(sx, sy, "og")
        plt.plot(gx, gy, "xb")
        plt.grid(True)
        plt.axis("equal")

    jps = JumpPointSearchPlanner(ox, oy, grid_size, robot_radius)
    rx, ry = jps.planning(sx, sy, gx, gy)

    if show_animation:  # pragma: no cover
        plt.plot(rx, ry, "-r")
        plt.pause(0.001)
        plt.show()


if __name__ == '__main__':
    main()
