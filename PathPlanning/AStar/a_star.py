"""

A* grid based planning

author: Atsushi Sakai(@Atsushi_twi)
        Nikos Kanargias (nkana@tee.gr)

See Wikipedia article (https://en.wikipedia.org/wiki/A*_search_algorithm)

"""

import matplotlib.pyplot as plt
import math

show_animation = True

class Node:

    def __init__(self, x, y, cost, pind):
        self.x = x # index of grid
        self.y = y # index of grid
        self.cost = cost
        self.pind = pind

    def __str__(self):
        return str(self.x) + "," + str(self.y) + "," + str(self.cost) + "," + str(self.pind)

def a_star_planning(sx, sy, gx, gy, ox, oy, reso, rr):
    """
    A star path search

    input:
        sx: start x position [m]
        sy: start y position [m]
        gx: goal x position [m]
        gx: goal x position [m]
        ox: x position list of Obstacles [m]
        oy: y position list of Obstacles [m]
        reso: grid resolution [m]
        rr: robot radius[m]

    output:
        rx: x position list of the final path
        ry: y position list of the final path
    """

    ox = [iox / reso for iox in ox]
    oy = [ioy / reso for ioy in oy]

    obmap, minx, miny, maxx, maxy, xw, yw = calc_obstacle_map(ox, oy, reso, rr)

    motion = get_motion_model()

    nstart = Node(calc_xyindex(sx, minx, reso), calc_xyindex(sy, minx, reso), 0.0, -1)
    ngoal = Node(calc_xyindex(gx, minx, reso), calc_xyindex(gy, minx, reso), 0.0, -1)

    openset, closedset = dict(), dict()
    openset[calc_index(nstart, xw, minx, miny)] = nstart

    while 1:
        c_id = min(
            openset, key=lambda o: openset[o].cost + calc_heuristic(ngoal, openset[o]))
        current = openset[c_id]

        # show graph
        if show_animation:  # pragma: no cover
            plt.plot(calc_position(current.x, minx, reso), calc_position(current.y, miny, reso), "xc")
            if len(closedset.keys()) % 10 == 0:
                plt.pause(0.001)

        if current.x == ngoal.x and current.y == ngoal.y:
            print("Find goal")
            ngoal.pind = current.pind
            ngoal.cost = current.cost
            break

        # Remove the item from the open set
        del openset[c_id]

        # Add it to the closed set
        closedset[c_id] = current

        # expand search grid based on motion model
        for i, _ in enumerate(motion):
            node = Node(current.x + motion[i][0],
                        current.y + motion[i][1],
                        current.cost + motion[i][2], c_id)
            n_id = calc_index(node, xw, minx, miny)

            if n_id in closedset:
                continue

            if not verify_node(node, obmap, minx, miny, maxx, maxy, reso):
                continue

            if n_id not in openset:
                openset[n_id] = node  # Discover a new node
            else:
                if openset[n_id].cost >= node.cost:
                    # This path is the best until now. record it!
                    openset[n_id] = node

    rx, ry = calc_final_path(ngoal, closedset, reso, minx, miny)

    return rx, ry



def calc_final_path(ngoal, closedset, reso, minx, miny):
    # generate final course
    rx, ry = [calc_position(ngoal.x, minx, reso)], [calc_position(ngoal.y, miny, reso)]
    pind = ngoal.pind
    while pind != -1:
        n = closedset[pind]
        rx.append(calc_position(n.x, minx, reso))
        ry.append(calc_position(n.y, miny, reso))
        pind = n.pind

    return rx, ry



def calc_heuristic(n1, n2):
    w = 1.0  # weight of heuristic
    d = w * math.sqrt((n1.x - n2.x)**2 + (n1.y - n2.y)**2)
    return d

def calc_position(index, minp, reso):
    return index*reso+minp

def calc_xyindex(position, minp, reso):
    return round((position - minp)/reso)

def verify_node(node, obmap, minx, miny, maxx, maxy, reso):

    px = calc_position(node.x, minx, reso)
    py = calc_position(node.y, miny, reso)

    if px < minx:
        return False
    elif py < miny:
        return False
    elif px >= maxx:
        return False
    elif py >= maxy:
        return False

    if obmap[node.x][node.y]:
        return False

    return True


def calc_obstacle_map(ox, oy, reso, rr):

    minx = round(min(ox))
    miny = round(min(oy))
    maxx = round(max(ox))
    maxy = round(max(oy))
    print("minx:", minx)
    print("miny:", miny)
    print("maxx:", maxx)
    print("maxy:", maxy)

    xwidth = round(maxx - minx)
    ywidth = round(maxy - miny)
    print("xwidth:", xwidth)
    print("ywidth:", ywidth)

    # obstacle map generation
    obmap = [[False for i in range(ywidth)] for i in range(xwidth)]
    for ix in range(xwidth):
        x = ix + minx
        for iy in range(ywidth):
            y = iy + miny
            for iox, ioy in zip(ox, oy):
                d = math.sqrt((iox - x)**2 + (ioy - y)**2)
                if d <= rr:
                    obmap[ix][iy] = True
                    break

    return obmap, minx, miny, maxx, maxy, xwidth, ywidth


def calc_index(node, xwidth, xmin, ymin):
    return (node.y - ymin) * xwidth + (node.x - xmin)


def get_motion_model():
    # dx, dy, cost
    motion = [[1, 0, 1],
              [0, 1, 1],
              [-1, 0, 1],
              [0, -1, 1],
              [-1, -1, math.sqrt(2)],
              [-1, 1, math.sqrt(2)],
              [1, -1, math.sqrt(2)],
              [1, 1, math.sqrt(2)]]

    return motion


def main():
    print(__file__ + " start!!")

    # start and goal position
    sx = 10.0  # [m]
    sy = 10.0  # [m]
    gx = 50.0  # [m]
    gy = 50.0  # [m]
    grid_size = 1.0  # [m]
    robot_radius = 1.0  # [m]

    # set obstable positions
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

    rx, ry = a_star_planning(sx, sy, gx, gy, ox, oy, grid_size, robot_radius)

    if show_animation:  # pragma: no cover
        plt.plot(rx, ry, "-r")
        plt.show()


if __name__ == '__main__':
    main()
