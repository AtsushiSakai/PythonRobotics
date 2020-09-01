'''
A* algorithm
Author: Weicent
randomly generate obstacles, start and goal point
searching path from start and end simultanously
'''

import numpy as np
import matplotlib.pyplot as plt
import math


class Node:
    '''Node with properties of G, H, Coordinate, Parent node'''

    def __init__(self, G=0, H=0, coordinate=None, parent=None):
        if coordinate is None:
            coordinate = [0, 0]
        self.G = G
        self.H = H
        self.F = G + H
        self.parent = parent
        self.coordinate = coordinate

    def setg(self, value):
        self.G = value

    def seth(self, value):
        self.H = value

    def setf(self):
        self.F = self.G + self.H

    def set_parent(self, new_parent):
        self.parent = new_parent


class NoPath(Exception):
    '''No path to the goal'''
    pass


class Break(Exception):
    '''Path is find, jump out of loop'''
    pass


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


def boundary_and_obstacles(start, goal, topv, botv, obstacle_number):
    '''
    start = start coordinate
    goal = goal coordinate
    topv = top vertex coordinate of boundary
    botv = bottom vertex coordinate of boundary
    obstacle_number = number of obstacles generated in the map
    '''
    # above can be merged into a rectangle boundary
    ay = list(range(botv[1], topv[1]))
    ax = [botv[0]] * len(ay)
    cy = ay
    cx = [topv[0]] * len(cy)
    bx = list(range(botv[0] + 1, topv[0]))
    by = [botv[1]] * len(bx)
    dx = [botv[0]] + bx + [topv[0]]
    dy = [topv[1]] * len(dx)

    # generate random obstacles
    ob_x = np.random.randint(botv[0] + 1, topv[0], obstacle_number).tolist()
    ob_y = np.random.randint(botv[1] + 1, topv[1], obstacle_number).tolist()
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
    flag = len(open_list)
    for i in range(flag):
        node = open_list[0]
        open_coor = [node.coordinate for node in open_list]
        closed_coor = [node.coordinate for node in closed_list]
        temp = find_neighbor(node, obstacle, closed_coor)
        for element in temp:
            if element in closed_list:
                continue
            elif element in open_coor:
                # if node in open list, update g value
                ind = open_coor.index(element)
                new_g = gcost(node, element)
                if new_g <= open_list[ind].G:
                    open_list[ind].setg(new_g)
                    open_list[ind].setf()
                    open_list[ind].set_parent = node
            else:  # new coordinate, create corresponding node
                ele_node = Node(coordinate=element, parent=node,
                                G=gcost(node, element), H=hcost(element, goal))
                open_list.append(ele_node)
        open_list.remove(node)
        closed_list.append(node)
        open_list.sort(key=lambda x: x.H)
    return open_list, closed_list


def draw_plot(close_origin, close_goal, start, end, bound):
    # plot the map
    plt.cla()
    plt.gcf().set_size_inches(12, 8, forward=True)
    plt.axis('equal')
    plt.plot(close_origin[:, 0], close_origin[:, 1], 'oy')
    plt.plot(close_goal[:, 0], close_goal[:, 1], 'og')
    plt.plot(bound[:, 0], bound[:, 1], 'sk')
    plt.plot(end[0], end[1], '*b', label='Goal')
    plt.plot(start[0], start[1], '^b', label='Origin')
    plt.legend()
    plt.pause(0.0001)


def node_to_coor(node_list):
    # convert node list into coordinate list and array
    coor_list = [node.coordinate for node in node_list]
    coor_array = np.array(coor_list)
    return coor_list, coor_array


def find_surrounding(coor, obs):
    # find obstacles around node, help to draw the borderline
    boundary: list = []
    for x in range(coor[0] - 1, coor[0] + 2):
        for y in range(coor[1] - 1, coor[1] + 2):
            if [x, y] in obs:
                boundary.append([x, y])
    return boundary


def border_line(closed, obs):
    # if no path, find border line which confine goal or robot
    border: list = []
    for node in closed:
        temp = find_surrounding(node, obs)
        border = border + temp
    border = np.array(border)
    return border


def get_path(org_list, goal_list, coor):
    # get path from start to end
    path_org: list = []
    path_goal: list = []
    ind = find_node_index(coor, org_list)
    node = org_list[ind]
    while node != org_list[0]:
        path_org.append(node.coordinate)
        node = node.parent
    path_org.append(org_list[0].coordinate)
    ind = find_node_index(coor, goal_list)
    node = goal_list[ind]
    while node != goal_list[0]:
        path_goal.append(node.coordinate)
        node = node.parent
    path_goal.append(goal_list[0].coordinate)
    path_org.reverse()
    path = path_org + path_goal
    path = np.array(path)
    return path


if __name__ == '__main__':
    try:
        try:
            topv = [60, 60]  # top vertex of boundary
            botv = [0, 0]  # bottoom vertex of boundary

            # generate start point randomly
            start = [np.random.randint(botv[0] + 1, topv[0]),
                     np.random.randint(botv[1] + 1, topv[1])]
            # generate goal randomly
            end = [np.random.randint(botv[0] + 1, topv[0]),
                   np.random.randint(botv[1] + 1, topv[1])]

            obstacle_number = 1500  # obstacle numbers

            # generate boundary and obstacles
            bound, obs = boundary_and_obstacles(start, end, topv,
                                                botv, obstacle_number)
            # initial origin node and end node
            origin = Node(coordinate=start, H=hcost(start, end))
            goal = Node(coordinate=end, H=hcost(end, start))
            # open and closed list for search from origin to goal
            origin_open = [origin]
            origin_close: list = []
            # open and closed list for search from goal to origin
            goal_open = [goal]
            goal_close: list = []
            # initialize coordinate closed list for origin and goal
            org_cor_list: list = []
            goa_cor_list: list = []
            # initialize searching target for origin and goal
            target_origin = start
            target_goal = end
            while True:
                # searching from start to end
                origin_open, origin_close = \
                    find_path(origin_open, origin_close, target_goal, bound)

                # convert node list into coordinate list and array
                org_cor_list, org_cor_array = node_to_coor(origin_close)

                if origin_open == []:  # no path condition
                    raise NoPath()

                # update target for searching from end to start
                target_origin = min(origin_open, key=lambda x: x.H).coordinate

                # searching from end to start
                goal_open, goal_close = \
                    find_path(goal_open, goal_close, target_origin, bound)

                # convert node list into coordinate list and array
                goa_cor_list, goa_cor_array = node_to_coor(goal_close)

                if goal_open == []:  # no path condition
                    raise NoPath()

                # update target for searching from start to end
                target_goal = \
                    min(goal_open, key=lambda x: x.H).coordinate

                # check if the searching meet each other
                og_intersect = \
                    [coor for coor in org_cor_list if coor in goa_cor_list]

                if og_intersect != []:  # a path is find
                    raise Break()

                draw_plot(org_cor_array, goa_cor_array, start, end, bound)

        except Break as success:
            # get path
            path = get_path(origin_close, goal_close, og_intersect[0])
            # plot map
            draw_plot(org_cor_array, goa_cor_array, start, end, bound)
            plt.plot(path[:, 0], path[:, 1], '-r')
            plt.title('Robot Arrived', size=20, loc='center')
    except NoPath as fail:
        if origin_open == []:
            # if origin confined, find border for origin
            border = border_line(org_cor_list, obs)
        elif goal_open == []:
            # if goal confined, find border for goal
            border = border_line(goa_cor_list, obs)
        if org_cor_list == []:
            # in case start point totally confined by obstacles
            # in this condition, no neighbor node generated at all,
            # and search ended with close_list=[]
            org_cor_list.append(start)
            org_cor_array = np.array(org_cor_list)
        if goa_cor_list == []:
            goa_cor_list.append(end)
            goa_cor_array = np.array(goa_cor_list)

        info = 'There is no path to  the goal!' \
               ' Robot&Goal are split by border' \
               ' shown in red \'x\'!'
        draw_plot(org_cor_array, goa_cor_array, start, end, bound)
        plt.plot(border[:, 0], border[:, 1], 'xr')
        plt.title(info, size=14, loc='center')

plt.show()
