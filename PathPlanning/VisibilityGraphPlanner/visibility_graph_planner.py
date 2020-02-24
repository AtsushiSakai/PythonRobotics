"""

Visibility Graph Planner

author: Atsushi Sakai (@Atsushi_twi)

"""

import matplotlib.pyplot as plt

show_animation = True

class VisibilityGraphPlanner:

    def __init__(self, robot_radius):
        self.robot_radius = robot_radius

    def planning(self, start_x, start_y, goal_x, goal_y, obstacles):
        nodes = self.exstract_graph_node(start_x, start_y, goal_x, goal_y,
                                         obstacles)

        graph = self.generate_graph(nodes, obstacles)

        rx, ry = dijkstra_search(graph)

        return rx, ry

    def exstract_graph_node(self, start_x, start_y, goal_x, goal_x, obstacles):
        nodes = []

        return nodes

    def generate_graph(self, nodes, obstacles):

        graph = []

        return graph


class ObstaclePolygon:

    def __init__(self, x_list, y_list):
        self.x_list = x_list
        self.y_list = y_list

        self.close_polygon()

    def close_polygon(self):
        is_x_same = self.x_list[0] == self.x_list[-1]
        is_y_same = self.y_list[0] == self.y_list[-1]
        if is_x_same and is_y_same:
            return  # no need to close

        self.x_list.append(self.x_list[0])
        self.y_list.append(self.y_list[0])

    def plot(self):
        plt.plot(self.x_list, self.y_list, "-b")


def main():
    print(__file__ + " start!!")

    # start and goal position
    sx, sy = 10.0, 10.0  # [m]
    gx, gy = 50.0, 50.0  # [m]
    robot_radius = 5.0  # [m]

    obstacles = [ObstaclePolygon(
        [20.0, 30.0, 15.0],
        [20.0, 20.0, 30.0],
    ), ObstaclePolygon(
        [30.0, 45.0, 50.0, 40.0],
        [50.0, 40.0, 20.0, 40.0],
    )]

    rx, ry = VisibilityGraphPlanner(robot_radius).planning(sx, sy, gx, gy,
                                                           obstacles)
    assert rx, 'Cannot found path'
    if show_animation:  # pragma: no cover
        plt.plot(sx, sy, "or")
        plt.plot(gx, gy, "ob")
        [ob.plot() for ob in obstacles]
        plt.plot(rx, ry, "-r")
        plt.axis("equal")
        plt.show()


if __name__ == '__main__':
    main()
