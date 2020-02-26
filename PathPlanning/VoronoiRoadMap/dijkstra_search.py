"""

Dijkstra Search library

author: Atsushi Sakai (@Atsushi_twi)

"""

import matplotlib.pyplot as plt
import math


class DijkstraSearch:

    class Node:
        """
        Node class for dijkstra search
        """

        def __init__(self, x, y, cost=None, parent=None):
            self.x = x
            self.y = y
            self.cost = cost
            self.parent = parent

        def __str__(self):
            return str(self.x) + "," + str(self.y) + "," + str(
                self.cost) + "," + str(self.parent)

    def __init__(self, show_animation):
        self.show_animation = show_animation

    def search(self, sx, sy, gx, gy, road_map, sample_x, sample_y):
        """
        gx: goal x position [m]
        gx: goal x position [m]
        ox: x position list of Obstacles [m]
        oy: y position list of Obstacles [m]
        reso: grid resolution [m]
        rr: robot radius[m]
        """

        start_node = self.Node(sx, sy, 0.0, -1)
        goal_node = self.Node(gx, gy, 0.0, -1)

        open_set, close_set = dict(), dict()
        open_set[len(road_map) - 2] = start_node

        while True:
            if not open_set:
                print("Cannot find path")
                break

            c_id = min(open_set, key=lambda o: open_set[o].cost)
            current = open_set[c_id]

            # show graph
            if self.show_animation and len(
                    close_set.keys()) % 2 == 0:  # pragma: no cover
                plt.plot(current.x, current.y, "xg")
                # for stopping simulation with the esc key.
                plt.gcf().canvas.mpl_connect(
                    'key_release_event',
                    lambda event: [exit(0) if event.key == 'escape' else None])
                plt.pause(0.001)

            if c_id == (len(road_map) - 1):
                print("goal is found!")
                goal_node.parent = current.parent
                goal_node.cost = current.cost
                break

            # Remove the item from the open set
            del open_set[c_id]
            # Add it to the closed set
            close_set[c_id] = current

            # expand search grid based on motion model
            for i in range(len(road_map[c_id])):
                n_id = road_map[c_id][i]
                dx = sample_x[n_id] - current.x
                dy = sample_y[n_id] - current.y
                d = math.hypot(dx, dy)
                node = self.Node(sample_x[n_id], sample_y[n_id],
                                 current.cost + d, c_id)

                if n_id in close_set:
                    continue
                # Otherwise if it is already in the open set
                if n_id in open_set:
                    if open_set[n_id].cost > node.cost:
                        open_set[n_id].cost = node.cost
                        open_set[n_id].parent = c_id
                else:
                    open_set[n_id] = node

        # generate final course
        rx, ry = [goal_node.x], [goal_node.y]
        parent = goal_node.parent
        while parent != -1:
            n = close_set[parent]
            rx.append(n.x)
            ry.append(n.y)
            parent = n.parent

        return rx, ry
