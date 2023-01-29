"""
A* algorithm
Author: Weicent
randomly generate obstacles, start and goal point
searching path from start and end simultaneously
"""

import numpy as np
import matplotlib.pyplot as plt
import math

show_animation = True


class Node:
    """node with properties of g, h, coordinate and parent node"""

    def __init__(self, G=0, H=0, coordinate=None, parent=None):
        self.G = G
        self.H = H
        self.F = G + H
        self.parent = parent
        self.coordinate = coordinate

    def reset_f(self):
        self.F = self.G + self.H


def hcost(node_coordinate, goal):
    dx = abs(node_coordinate[0] - goal[0])
    dy = abs(node_coordinate[1] - goal[1])
    hcost = dx + dy
    return hcost


def gcost(fixed_node, update_node_coordinate):
    dx = abs(fixed_node.coordinate[0] - update_node_coordinate[0])
    dy = abs(fixed_node.coordinate[1] - update_node_coordinate[1])
    gc = math.hypot(dx, dy)  # gc = move from fixed_node to update_node
    gcost = fixed_node.G + gc  # gcost = move from start point to update_node
    return gcost


def boundary_and_obstacles(start, goal, top_vertex, bottom_vertex, obs_number):
    """
    :param start: start coordinate
    :param goal: goal coordinate
    :param top_vertex: top right vertex coordinate of boundary
    :param bottom_vertex: bottom left vertex coordinate of boundary
    :param obs_number: number of obstacles generated in the map
    :return: boundary_obstacle array, obstacle list
    """
    # below can be merged into a rectangle boundary
    ay = list(range(bottom_vertex[1], top_vertex[1]))
    ax = [bottom_vertex[0]] * len(ay)
    cy = ay
    cx = [top_vertex[0]] * len(cy)
    bx = list(range(bottom_vertex[0] + 1, top_vertex[0]))
    by = [bottom_vertex[1]] * len(bx)
    dx = [bottom_vertex[0]] + bx + [top_vertex[0]]
    dy = [top_vertex[1]] * len(dx)

    # generate random obstacles
    ob_x = np.random.randint(bottom_vertex[0] + 1,
                             top_vertex[0], obs_number).tolist()
    ob_y = np.random.randint(bottom_vertex[1] + 1,
                             top_vertex[1], obs_number).tolist()
    # x y coordinate in certain order for boundary
    x = ax + bx + cx + dx
    y = ay + by + cy + dy
    obstacle = np.vstack((ob_x, ob_y)).T.tolist()
    # remove start and goal coordinate in obstacle list
    obstacle = [coor for coor in obstacle if coor != start and coor != goal]
    obs_array = np.array(obstacle)
    bound = np.vstack((x, y)).T
    bound_obs = np.vstack((bound, obs_array))
    return bound_obs, obstacle


def find_neighbor(node, ob, closed):
    # generate neighbors in certain condition
    ob_list = ob.tolist()
    neighbor: list = []
    for x in range(node.coordinate[0] - 1, node.coordinate[0] + 2):
        for y in range(node.coordinate[1] - 1, node.coordinate[1] + 2):
            if [x, y] not in ob_list:
                # find all possible neighbor nodes
                neighbor.append([x, y])
    # remove node violate the motion rule
    # 1. remove node.coordinate itself
    neighbor.remove(node.coordinate)
    # 2. remove neighbor nodes who cross through two diagonal
    # positioned obstacles since there is no enough space for
    # robot to go through two diagonal positioned obstacles

    # top bottom left right neighbors of node
    top_nei = [node.coordinate[0], node.coordinate[1] + 1]
    bottom_nei = [node.coordinate[0], node.coordinate[1] - 1]
    left_nei = [node.coordinate[0] - 1, node.coordinate[1]]
    right_nei = [node.coordinate[0] + 1, node.coordinate[1]]
    # neighbors in four vertex
    lt_nei = [node.coordinate[0] - 1, node.coordinate[1] + 1]
    rt_nei = [node.coordinate[0] + 1, node.coordinate[1] + 1]
    lb_nei = [node.coordinate[0] - 1, node.coordinate[1] - 1]
    rb_nei = [node.coordinate[0] + 1, node.coordinate[1] - 1]

    # remove the unnecessary neighbors
    if top_nei and left_nei in ob_list and lt_nei in neighbor:
        neighbor.remove(lt_nei)
    if top_nei and right_nei in ob_list and rt_nei in neighbor:
        neighbor.remove(rt_nei)
    if bottom_nei and left_nei in ob_list and lb_nei in neighbor:
        neighbor.remove(lb_nei)
    if bottom_nei and right_nei in ob_list and rb_nei in neighbor:
        neighbor.remove(rb_nei)
    neighbor = [x for x in neighbor if x not in closed]
    return neighbor


def find_node_index(coordinate, node_list):
    # find node index in the node list via its coordinate
    ind = 0
    for node in node_list:
        if node.coordinate == coordinate:
            target_node = node
            ind = node_list.index(target_node)
            break
    return ind


def find_path(open_list, closed_list, goal, obstacle):
    # searching for the path, update open and closed list
    # obstacle = obstacle and boundary
    flag = len(open_list)
    for i in range(flag):
        node = open_list[0]
        open_coordinate_list = [node.coordinate for node in open_list]
        closed_coordinate_list = [node.coordinate for node in closed_list]
        temp = find_neighbor(node, obstacle, closed_coordinate_list)
        for element in temp:
            if element in closed_list:
                continue
            elif element in open_coordinate_list:
                # if node in open list, update g value
                ind = open_coordinate_list.index(element)
                new_g = gcost(node, element)
                if new_g <= open_list[ind].G:
                    open_list[ind].G = new_g
                    open_list[ind].reset_f()
                    open_list[ind].parent = node
            else:  # new coordinate, create corresponding node
                ele_node = Node(coordinate=element, parent=node,
                                G=gcost(node, element), H=hcost(element, goal))
                open_list.append(ele_node)
        open_list.remove(node)
        closed_list.append(node)
        open_list.sort(key=lambda x: x.F)
    return open_list, closed_list


def node_to_coordinate(node_list):
    # convert node list into coordinate list and array
    coordinate_list = [node.coordinate for node in node_list]
    return coordinate_list


def check_node_coincide(close_ls1, closed_ls2):
    """
    :param close_ls1: node closed list for searching from start
    :param closed_ls2: node closed list for searching from end
    :return: intersect node list for above two
    """
    # check if node in close_ls1 intersect with node in closed_ls2
    cl1 = node_to_coordinate(close_ls1)
    cl2 = node_to_coordinate(closed_ls2)
    intersect_ls = [node for node in cl1 if node in cl2]
    return intersect_ls


def find_surrounding(coordinate, obstacle):
    # find obstacles around node, help to draw the borderline
    boundary: list = []
    for x in range(coordinate[0] - 1, coordinate[0] + 2):
        for y in range(coordinate[1] - 1, coordinate[1] + 2):
            if [x, y] in obstacle:
                boundary.append([x, y])
    return boundary


def get_border_line(node_closed_ls, obstacle):
    # if no path, find border line which confine goal or robot
    border: list = []
    coordinate_closed_ls = node_to_coordinate(node_closed_ls)
    for coordinate in coordinate_closed_ls:
        temp = find_surrounding(coordinate, obstacle)
        border = border + temp
    border_ary = np.array(border)
    return border_ary


def get_path(org_list, goal_list, coordinate):
    # get path from start to end
    path_org: list = []
    path_goal: list = []
    ind = find_node_index(coordinate, org_list)
    node = org_list[ind]
    while node != org_list[0]:
        path_org.append(node.coordinate)
        node = node.parent
    path_org.append(org_list[0].coordinate)
    ind = find_node_index(coordinate, goal_list)
    node = goal_list[ind]
    while node != goal_list[0]:
        path_goal.append(node.coordinate)
        node = node.parent
    path_goal.append(goal_list[0].coordinate)
    path_org.reverse()
    path = path_org + path_goal
    path = np.array(path)
    return path


def random_coordinate(bottom_vertex, top_vertex):
    # generate random coordinates inside maze
    coordinate = [np.random.randint(bottom_vertex[0] + 1, top_vertex[0]),
                  np.random.randint(bottom_vertex[1] + 1, top_vertex[1])]
    return coordinate


def draw(close_origin, close_goal, start, end, bound):
    # plot the map
    if not close_goal.tolist():  # ensure the close_goal not empty
        # in case of the obstacle number is really large (>4500), the
        # origin is very likely blocked at the first search, and then
        # the program is over and the searching from goal to origin
        # will not start, which remain the closed_list for goal == []
        # in order to plot the map, add the end coordinate to array
        close_goal = np.array([end])
    plt.cla()
    plt.gcf().set_size_inches(11, 9, forward=True)
    plt.axis('equal')
    plt.plot(close_origin[:, 0], close_origin[:, 1], 'oy')
    plt.plot(close_goal[:, 0], close_goal[:, 1], 'og')
    plt.plot(bound[:, 0], bound[:, 1], 'sk')
    plt.plot(end[0], end[1], '*b', label='Goal')
    plt.plot(start[0], start[1], '^b', label='Origin')
    plt.legend()
    plt.pause(0.0001)


def draw_control(org_closed, goal_closed, flag, start, end, bound, obstacle):
    """
    control the plot process, evaluate if the searching finished
    flag == 0 : draw the searching process and plot path
    flag == 1 or 2 : start or end is blocked, draw the border line
    """
    stop_loop = 0  # stop sign for the searching
    org_closed_ls = node_to_coordinate(org_closed)
    org_array = np.array(org_closed_ls)
    goal_closed_ls = node_to_coordinate(goal_closed)
    goal_array = np.array(goal_closed_ls)
    path = None
    if show_animation:  # draw the searching process
        draw(org_array, goal_array, start, end, bound)
    if flag == 0:
        node_intersect = check_node_coincide(org_closed, goal_closed)
        if node_intersect:  # a path is find
            path = get_path(org_closed, goal_closed, node_intersect[0])
            stop_loop = 1
            print('Path found!')
            if show_animation:  # draw the path
                plt.plot(path[:, 0], path[:, 1], '-r')
                plt.title('Robot Arrived', size=20, loc='center')
                plt.pause(0.01)
                plt.show()
    elif flag == 1:  # start point blocked first
        stop_loop = 1
        print('There is no path to the goal! Start point is blocked!')
    elif flag == 2:  # end point blocked first
        stop_loop = 1
        print('There is no path to the goal! End point is blocked!')
    if show_animation:  # blocked case, draw the border line
        info = 'There is no path to the goal!' \
               ' Robot&Goal are split by border' \
               ' shown in red \'x\'!'
        if flag == 1:
            border = get_border_line(org_closed, obstacle)
            plt.plot(border[:, 0], border[:, 1], 'xr')
            plt.title(info, size=14, loc='center')
            plt.pause(0.01)
            plt.show()
        elif flag == 2:
            border = get_border_line(goal_closed, obstacle)
            plt.plot(border[:, 0], border[:, 1], 'xr')
            plt.title(info, size=14, loc='center')
            plt.pause(0.01)
            plt.show()
    return stop_loop, path


def searching_control(start, end, bound, obstacle):
    """manage the searching process, start searching from two side"""
    # initial origin node and end node
    origin = Node(coordinate=start, H=hcost(start, end))
    goal = Node(coordinate=end, H=hcost(end, start))
    # list for searching from origin to goal
    origin_open: list = [origin]
    origin_close: list = []
    # list for searching from goal to origin
    goal_open = [goal]
    goal_close: list = []
    # initial target
    target_goal = end
    # flag = 0 (not blocked) 1 (start point blocked) 2 (end point blocked)
    flag = 0  # init flag
    path = None
    while True:
        # searching from start to end
        origin_open, origin_close = \
            find_path(origin_open, origin_close, target_goal, bound)
        if not origin_open:  # no path condition
            flag = 1  # origin node is blocked
            draw_control(origin_close, goal_close, flag, start, end, bound,
                         obstacle)
            break
        # update target for searching from end to start
        target_origin = min(origin_open, key=lambda x: x.F).coordinate

        # searching from end to start
        goal_open, goal_close = \
            find_path(goal_open, goal_close, target_origin, bound)
        if not goal_open:  # no path condition
            flag = 2  # goal is blocked
            draw_control(origin_close, goal_close, flag, start, end, bound,
                         obstacle)
            break
        # update target for searching from start to end
        target_goal = min(goal_open, key=lambda x: x.F).coordinate

        # continue searching, draw the process
        stop_sign, path = draw_control(origin_close, goal_close, flag, start,
                                       end, bound, obstacle)
        if stop_sign:
            break
    return path


def main(obstacle_number=1500):
    print(__file__ + ' start!')

    top_vertex = [60, 60]  # top right vertex of boundary
    bottom_vertex = [0, 0]  # bottom left vertex of boundary

    # generate start and goal point randomly
    start = random_coordinate(bottom_vertex, top_vertex)
    end = random_coordinate(bottom_vertex, top_vertex)

    # generate boundary and obstacles
    bound, obstacle = boundary_and_obstacles(start, end, top_vertex,
                                             bottom_vertex,
                                             obstacle_number)

    path = searching_control(start, end, bound, obstacle)
    if not show_animation:
        print(path)


if __name__ == '__main__':
    main(obstacle_number=1500)
