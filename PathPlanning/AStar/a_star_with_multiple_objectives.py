"""

Multiple Objectives A* grid planning

author: Robin Mann (robinmann13)
        Jan Luca Frank (JanLucaF)
        Julia Stein (JuliaCStein)

"""

import math
import matplotlib.pyplot as plt
import numpy as np

show_animation = True

class AStarPlanner:

    def __init__(self, ox, oy, resolution, rr, path_length_weight=0.0, distance_weight=0.0, energy_cost_weight=0.0):
        """
        Initialize grid map for a star planning

        ox: x position list of Obstacles [m]
        oy: y position list of Obstacles [m]
        resolution: grid resolution [m]
        rr: robot radius[m]
        path_length_weight: weight for the path length
        distance_weight: weight for the distance
        energy_cost_weight: weight for the energy cost
        """
        self.resolution = resolution
        self.rr = rr
        self.min_x, self.min_y = 0, 0
        self.max_x, self.max_y = 0, 0
        self.obstacle_map = None
        self.distance_map = None
        self.path_length_weight = path_length_weight
        self.distance_weight = distance_weight
        self.energy_cost_weight = energy_cost_weight
        self.x_width, self.y_width = 0, 0
        self.motion = self.get_motion_model()
        self.calc_obstacle_map(ox, oy)

    class Node:
        def __init__(self, x, y, cost, parent_index):
            self.x = x
            self.y = y
            self.cost = cost
            self.parent_index = parent_index

        def __str__(self):
            return f"{self.x},{self.y},{self.cost},{self.parent_index}"

    def planning(self, sx, sy, gx, gy):
        """
        A star path search

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

        open_set, closed_set = dict(), dict()
        open_set[self.calc_grid_index(start_node)] = start_node

        while True:
            if len(open_set) == 0:
                print("Open set is empty..")
                break

            c_id = min(
                open_set,
                key=lambda o: open_set[o].cost + self.calc_distance_penalty(open_set[o].x, open_set[o].y) + self.calc_heuristic(goal_node, open_set[o]))
            current = open_set[c_id]

            # show graph
            if show_animation:  # pragma: no cover
                plt.plot(self.calc_grid_position(current.x, self.min_x),
                         self.calc_grid_position(current.y, self.min_y), "xc")
                # for stopping simulation with the esc key.
                plt.gcf().canvas.mpl_connect('key_release_event',
                                             lambda event: [exit(0) if event.key == 'escape' else None])
                if len(closed_set.keys()) % 100000 == 0:
                    plt.pause(0.01)

            if current.x == goal_node.x and current.y == goal_node.y:
                print("Find goal")
                goal_node.parent_index = current.parent_index
                goal_node.cost = current.cost
                break
            
            # Remove the item from the open set
            del open_set[c_id]

            # Add it to the closed set
            closed_set[c_id] = current

            # expand_grid search grid based on motion model
            for i, _ in enumerate(self.motion):
                move_cost = self.motion[i][2]*(self.path_length_weight)
                neighbor_x = current.x + self.motion[i][0]
                neighbor_y = current.y + self.motion[i][1]
                energy_cost = self.calc_energy_cost(current, self.Node(neighbor_x, neighbor_y, 0, -1), closed_set)

                node = self.Node(current.x + self.motion[i][0],
                                 current.y + self.motion[i][1],
                                 current.cost + move_cost + energy_cost,
                                 c_id)

                n_id = self.calc_grid_index(node)

                # If the node is not safe, do nothing
                if not self.verify_node(node):
                    continue

                if n_id in closed_set:
                    continue

                if n_id not in open_set:
                    open_set[n_id] = node   # discovered a new node
                else:
                    if open_set[n_id].cost > node.cost:
                        # This path is the best until now. record it
                        open_set[n_id] = node

        rx, ry = self.calc_final_path(goal_node, closed_set)
        return rx, ry
    
    def calc_distance_penalty(self, x, y):
        if self.distance_map is None:
            return 0.0

        # Calculate the maximum distance (the largest distance in the map)
        max_distance = np.max(self.distance_map) 

        if 0 <= x < self.distance_map.shape[0] and 0 <= y < self.distance_map.shape[1]:
            # Get the distance of the current node from the distance map
            distance = self.distance_map[x, y]
        else:
            return 0.0
        
        # Get the distance of the neighbors of the current node
        neighbor_clearances = []
        for i, _ in enumerate(self.motion):
            nx, ny = x + self.motion[i][0], y + self.motion[i][1]
            if 0 <= nx < self.distance_map.shape[0] and 0 <= ny < self.distance_map.shape[1]:
                neighbor_clearances.append(self.distance_map[nx, ny])
 
        # Calculate the minimum distance of the neighbors
        min_neighbor_clearance = min(neighbor_clearances) if neighbor_clearances else distance
 
        # Calculate the combined clearance of the current node and its neighbors
        cost_weight_neigbours = 0.7  # weight of the neighbors' clearance (0.7)
        combined_clearance = cost_weight_neigbours * distance + (1 - cost_weight_neigbours) * min_neighbor_clearance
         # Normalize the combined clearance between 0 and 100
        penalty = (100.0 * (1 - combined_clearance / max_distance)* self.distance_weight)
        return penalty

    def calc_energy_cost(self, current, neighbor, closed_set):
        parent_node = closed_set.get(current.parent_index, None)
        if parent_node is None:
            return 0.0

        # Calculate the vectors between parent and current node
        prev_dx = current.x - parent_node.x
        prev_dy = current.y - parent_node.y

        # Calculate the vectors between current and neighbor node
        current_dx = neighbor.x - current.x
        current_dy = neighbor.y - current.y

        # Calculate the dot product of the two vectors
        dot_product = prev_dx * current_dx + prev_dy * current_dy
        prev_length = math.sqrt(prev_dx ** 2 + prev_dy ** 2)
        current_length = math.sqrt(current_dx ** 2 + current_dy ** 2)

        # Calculate the cosine of the angle between the two vectors
        cos_theta = dot_product / (prev_length * current_length + 1e-6)
        angle = math.acos(max(-1.0, min(1.0, cos_theta)))

        max_angle = math.pi
        normalized_energy_cost = 100 * abs(angle) / max_angle

        return abs(normalized_energy_cost) * self.energy_cost_weight

    def calc_final_path(self, goal_node, closed_set):
        # generate final course
        rx, ry = [self.calc_grid_position(goal_node.x, self.min_x)], [
            self.calc_grid_position(goal_node.y, self.min_y)]
        parent_index = goal_node.parent_index
        while parent_index != -1:
            n = closed_set[parent_index]
            rx.append(self.calc_grid_position(n.x, self.min_x))
            ry.append(self.calc_grid_position(n.y, self.min_y))
            parent_index = n.parent_index

        return rx, ry

    @staticmethod
    def calc_heuristic(n1, n2):
        w = 1.0 # weigth of heuristic
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

    def verify_node(self, node):
        px = self.calc_grid_position(node.x, self.min_x)
        py = self.calc_grid_position(node.y, self.min_y)

        if px < self.min_x or py < self.min_y or px >= self.max_x or py >= self.max_y:
            return False
        if self.obstacle_map[node.x][node.y]:
            return False

        return True

    def calc_obstacle_map(self, ox, oy): # obstacle map and distance map generation
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
        # distance map generation
        self.distance_map = np.full((self.x_width, self.y_width), float('inf'))

        for ix in range(self.x_width):
            x = self.calc_grid_position(ix, self.min_x)
            for iy in range(self.y_width):
                y = self.calc_grid_position(iy, self.min_y)
                min_distance = float('inf')
                for iox, ioy in zip(ox, oy):
                    # Euklidische Distanz:
                    d = math.hypot(iox - x, ioy - y)
                    # Manhattan-Distanz:
                    #d = abs(iox - x) + abs(ioy - y)
                    min_distance = min(min_distance, d)
                    if d <= self.rr:
                        self.obstacle_map[ix][iy] = True
                        break
                self.distance_map[ix, iy] = max(0, min_distance - self.rr)

    @staticmethod
    def get_motion_model():
        # dx, dy, move cost
        return [[1, 0, 1],
                [0, 1, 1],
                [-1, 0, 1],
                [0, -1, 1],
                [-1, -1, math.sqrt(2)],
                [-1, 1, math.sqrt(2)],
                [1, -1, math.sqrt(2)],
                [1, 1, math.sqrt(2)]]

def main():
    print(__file__ + " start!!")

    # start and goal position
    sx, sy = 5.0, 10.0
    gx, gy = 55.0, 55.0
    grid_size = 2.0
    robot_radius = 1.0
    path_length_weight = 1
    distance_weight = 1
    energy_cost_weight = 0.1

    ox, oy = [], []
    for i in range(0, 60):
        ox.append(i)
        oy.append(0.0)
    for i in range(0, 60):
        ox.append(60.0)
        oy.append(i)
    for i in range(0, 61):
        ox.append(i)
        oy.append(60.0)
    for i in range(0, 61):
        ox.append(0.0)
        oy.append(i)
    for i in range(0, 40):
        ox.append(10.0)
        oy.append(i)
    for i in range(0, 40):
        ox.append(30.0)
        oy.append(i)
    for i in range(0, 40):
        ox.append(20.0)
        oy.append(60.0 - i)
    for i in range(0, 40):
        ox.append(40.0)
        oy.append(60.0 - i)

    if show_animation:  # pragma: no cover
        plt.figure()
        plt.plot(ox, oy, ".k", label="Obstacles")
        plt.plot(sx, sy, "og", label="Start")
        plt.plot(gx, gy, "xb", label="Goal")
        plt.grid(True)
        plt.axis("equal")
        
    a_star = AStarPlanner(ox, oy, grid_size, robot_radius, path_length_weight, distance_weight, energy_cost_weight)
    rx, ry = a_star.planning(sx, sy, gx, gy)

    if show_animation:  # pragma: no cover
        plt.plot(rx, ry, "-r")
        plt.pause(0.001)
    plt.show()

if __name__ == '__main__':
    main()
