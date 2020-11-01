"""
a star variants
author: Sarim Mehdi(muhammadsarim.mehdi@studio.unibo.it)
Source: http://theory.stanford.edu/~amitp/GameProgramming/Variations.html
"""

import numpy as np
import matplotlib.pyplot as plt

show_animation = True
use_beam_search = False
use_iterative_deepening = False
use_dynamic_weighting = False
use_theta_star = False
use_jump_point = False

beam_capacity = 30
max_theta = 5
only_corners = False
max_corner = 5
w, epsilon, upper_bound_depth = 1, 4, 500


def draw_horizontal_line(start_x, start_y, length, o_x, o_y, o_dict):
    for i in range(start_x, start_x + length):
        for j in range(start_y, start_y + 2):
            o_x.append(i)
            o_y.append(j)
            o_dict[(i, j)] = True


def draw_vertical_line(start_x, start_y, length, o_x, o_y, o_dict):
    for i in range(start_x, start_x + 2):
        for j in range(start_y, start_y + length):
            o_x.append(i)
            o_y.append(j)
            o_dict[(i, j)] = True


def in_line_of_sight(obs_grid, x1, y1, x2, y2):
    t = 0
    while t <= 0.5:
        xt = (1 - t) * x1 + t * x2
        yt = (1 - t) * y1 + t * y2
        if obs_grid[(int(xt), int(yt))]:
            return False, None
        xt = (1 - t) * x2 + t * x1
        yt = (1 - t) * y2 + t * y1
        if obs_grid[(int(xt), int(yt))]:
            return False, None
        t += 0.001
    dist = np.linalg.norm(np.array([x1, y1] - np.array([x2, y2])))
    return True, dist


def key_points(o_dict):
    offsets1 = [(1, 0), (0, 1), (-1, 0), (1, 0)]
    offsets2 = [(1, 1), (-1, 1), (-1, -1), (1, -1)]
    offsets3 = [(0, 1), (-1, 0), (0, -1), (0, -1)]
    c_list = []
    for grid_point, obs_status in o_dict.items():
        if obs_status:
            continue
        empty_space = True
        x, y = grid_point
        for i in [-1, 0, 1]:
            for j in [-1, 0, 1]:
                if (x + i, y + j) not in o_dict.keys():
                    continue
                if o_dict[(x + i, y + j)]:
                    empty_space = False
                    break
        if empty_space:
            continue
        for offset1, offset2, offset3 in zip(offsets1, offsets2, offsets3):
            i1, j1 = offset1
            i2, j2 = offset2
            i3, j3 = offset3
            if ((x + i1, y + j1) not in o_dict.keys()) or \
                    ((x + i2, y + j2) not in o_dict.keys()) or \
                    ((x + i3, y + j3) not in o_dict.keys()):
                continue
            obs_count = 0
            if o_dict[(x + i1, y + j1)]:
                obs_count += 1
            if o_dict[(x + i2, y + j2)]:
                obs_count += 1
            if o_dict[(x + i3, y + j3)]:
                obs_count += 1
            if obs_count == 3 or obs_count == 1:
                c_list.append((x, y))
                if show_animation:
                    plt.plot(x, y, ".y")
                break
    if only_corners:
        return c_list

    e_list = []
    for corner in c_list:
        x1, y1 = corner
        for other_corner in c_list:
            x2, y2 = other_corner
            if x1 == x2 and y1 == y2:
                continue
            reachable, _ = in_line_of_sight(o_dict, x1, y1, x2, y2)
            if not reachable:
                continue
            x_m, y_m = int((x1 + x2) / 2), int((y1 + y2) / 2)
            e_list.append((x_m, y_m))
            if show_animation:
                plt.plot(x_m, y_m, ".y")
    return c_list + e_list


class SearchAlgo:
    def __init__(self, obs_grid, goal_x, goal_y, start_x, start_y,
                 limit_x, limit_y, corner_list=None):
        self.start_pt = [start_x, start_y]
        self.goal_pt = [goal_x, goal_y]
        self.obs_grid = obs_grid
        g_cost, h_cost = 0, self.get_hval(start_x, start_y, goal_x, goal_y)
        f_cost = g_cost + h_cost
        self.all_nodes, self.open_set = {}, []

        if use_jump_point:
            for corner in corner_list:
                i, j = corner
                h_c = self.get_hval(i, j, goal_x, goal_y)
                self.all_nodes[(i, j)] = {'pos': [i, j], 'pred': None,
                                          'gcost': np.inf, 'hcost': h_c,
                                          'fcost': np.inf,
                                          'open': True, 'in_open_list': False}
            self.all_nodes[tuple(self.goal_pt)] = \
                {'pos': self.goal_pt, 'pred': None,
                 'gcost': np.inf, 'hcost': 0, 'fcost': np.inf,
                 'open': True, 'in_open_list': True}
        else:
            for i in range(limit_x):
                for j in range(limit_y):
                    h_c = self.get_hval(i, j, goal_x, goal_y)
                    self.all_nodes[(i, j)] = {'pos': [i, j], 'pred': None,
                                              'gcost': np.inf, 'hcost': h_c,
                                              'fcost': np.inf,
                                              'open': True,
                                              'in_open_list': False}
        self.all_nodes[tuple(self.start_pt)] = \
            {'pos': self.start_pt, 'pred': None,
             'gcost': g_cost, 'hcost': h_cost, 'fcost': f_cost,
             'open': True, 'in_open_list': True}
        self.open_set.append(self.all_nodes[tuple(self.start_pt)])

    @staticmethod
    def get_hval(x1, y1, x2, y2):
        x, y = x1, y1
        val = 0
        while x != x2 or y != y2:
            if x != x2 and y != y2:
                val += 14
            else:
                val += 10
            x, y = x + np.sign(x2 - x), y + np.sign(y2 - y)
        return val

    def get_farthest_point(self, x, y, i, j):
        i_temp, j_temp = i, j
        counter = 1
        got_goal = False
        while not self.obs_grid[(x + i_temp, y + j_temp)] and \
                counter < max_theta:
            i_temp += i
            j_temp += j
            counter += 1
            if [x + i_temp, y + j_temp] == self.goal_pt:
                got_goal = True
                break
            if (x + i_temp, y + j_temp) not in self.obs_grid.keys():
                break
        return i_temp - 2*i, j_temp - 2*j, counter, got_goal

    def jump_point(self):
        """Jump point: Instead of exploring all empty spaces of the
        map, just explore the corners."""

        goal_found = False
        while len(self.open_set) > 0:
            self.open_set = sorted(self.open_set, key=lambda x: x['fcost'])
            lowest_f = self.open_set[0]['fcost']
            lowest_h = self.open_set[0]['hcost']
            lowest_g = self.open_set[0]['gcost']
            p = 0
            for p_n in self.open_set[1:]:
                if p_n['fcost'] == lowest_f and \
                        p_n['gcost'] < lowest_g:
                    lowest_g = p_n['gcost']
                    p += 1
                elif p_n['fcost'] == lowest_f and \
                        p_n['gcost'] == lowest_g and \
                        p_n['hcost'] < lowest_h:
                    lowest_h = p_n['hcost']
                    p += 1
                else:
                    break
            current_node = self.all_nodes[tuple(self.open_set[p]['pos'])]
            x1, y1 = current_node['pos']

            for cand_pt, cand_node in self.all_nodes.items():
                x2, y2 = cand_pt
                if x1 == x2 and y1 == y2:
                    continue
                if np.linalg.norm(np.array([x1, y1] -
                                           np.array([x2, y2]))) > max_corner:
                    continue
                reachable, offset = in_line_of_sight(self.obs_grid, x1,
                                                     y1, x2, y2)
                if not reachable:
                    continue

                if list(cand_pt) == self.goal_pt:
                    current_node['open'] = False
                    self.all_nodes[tuple(cand_pt)]['pred'] = \
                        current_node['pos']
                    goal_found = True
                    break

                g_cost = offset + current_node['gcost']
                h_cost = self.all_nodes[cand_pt]['hcost']
                f_cost = g_cost + h_cost
                cand_pt = tuple(cand_pt)
                if f_cost < self.all_nodes[cand_pt]['fcost']:
                    self.all_nodes[cand_pt]['pred'] = current_node['pos']
                    self.all_nodes[cand_pt]['gcost'] = g_cost
                    self.all_nodes[cand_pt]['fcost'] = f_cost
                    if not self.all_nodes[cand_pt]['in_open_list']:
                        self.open_set.append(self.all_nodes[cand_pt])
                        self.all_nodes[cand_pt]['in_open_list'] = True
                    if show_animation:
                        plt.plot(cand_pt[0], cand_pt[1], "r*")

                if goal_found:
                    break
            if show_animation:
                plt.pause(0.001)
            if goal_found:
                current_node = self.all_nodes[tuple(self.goal_pt)]
            while goal_found:
                if current_node['pred'] is None:
                    break
                x = [current_node['pos'][0], current_node['pred'][0]]
                y = [current_node['pos'][1], current_node['pred'][1]]
                current_node = self.all_nodes[tuple(current_node['pred'])]
                if show_animation:
                    plt.plot(x, y, "b")
                    plt.pause(0.001)
            if goal_found:
                break

            current_node['open'] = False
            current_node['in_open_list'] = False
            if show_animation:
                plt.plot(current_node['pos'][0], current_node['pos'][1], "g*")
            del self.open_set[p]
            current_node['fcost'], current_node['hcost'] = np.inf, np.inf
        if show_animation:
            plt.title('Jump Point')
            plt.show()

    def a_star(self):
        """Beam search: Maintain an open list of just 30 nodes.
        If more than 30 nodes, then get rid of nodes with high
        f values.
        Iterative deepening: At every iteration, get a cut-off
        value for the f cost. This cut-off is minimum of the f
        value of all nodes whose f value is higher than the
        current cut-off value. Nodes with f value higher than
        the current cut off value are not put in the open set.
        Dynamic weighting: Multiply heuristic with the following:
        (1 + epsilon - (epsilon*d)/N) where d is the current
        iteration of loop and N is upper bound on number of
        iterations.
        Theta star: Same as A star but you don't need to move
        one neighbor at a time. In fact, you can look for the
        next node as far out as you can as long as there is a
        clear line of sight from your current node to that node."""
        if show_animation:
            if use_beam_search:
                plt.title('A* with beam search')
            elif use_iterative_deepening:
                plt.title('A* with iterative deepening')
            elif use_dynamic_weighting:
                plt.title('A* with dynamic weighting')
            elif use_theta_star:
                plt.title('Theta*')
            else:
                plt.title('A*')

        goal_found = False
        curr_f_thresh = np.inf
        depth = 0
        no_valid_f = False
        w = None
        while len(self.open_set) > 0:
            self.open_set = sorted(self.open_set, key=lambda x: x['fcost'])
            lowest_f = self.open_set[0]['fcost']
            lowest_h = self.open_set[0]['hcost']
            lowest_g = self.open_set[0]['gcost']
            p = 0
            for p_n in self.open_set[1:]:
                if p_n['fcost'] == lowest_f and \
                        p_n['gcost'] < lowest_g:
                    lowest_g = p_n['gcost']
                    p += 1
                elif p_n['fcost'] == lowest_f and \
                        p_n['gcost'] == lowest_g and \
                        p_n['hcost'] < lowest_h:
                    lowest_h = p_n['hcost']
                    p += 1
                else:
                    break
            current_node = self.all_nodes[tuple(self.open_set[p]['pos'])]

            while len(self.open_set) > beam_capacity and use_beam_search:
                del self.open_set[-1]

            f_cost_list = []
            if use_dynamic_weighting:
                w = (1 + epsilon - epsilon*depth/upper_bound_depth)
            for i in range(-1, 2):
                for j in range(-1, 2):
                    x, y = current_node['pos']
                    if (i == 0 and j == 0) or \
                            ((x + i, y + j) not in self.obs_grid.keys()):
                        continue
                    if (i, j) in [(1, 0), (0, 1), (-1, 0), (0, -1)]:
                        offset = 10
                    else:
                        offset = 14
                    if use_theta_star:
                        new_i, new_j, counter, goal_found = \
                            self.get_farthest_point(x, y, i, j)
                        offset = offset * counter
                        cand_pt = [current_node['pos'][0] + new_i,
                                   current_node['pos'][1] + new_j]
                    else:
                        cand_pt = [current_node['pos'][0] + i,
                                   current_node['pos'][1] + j]

                    if use_theta_star and goal_found:
                        current_node['open'] = False
                        cand_pt = self.goal_pt
                        self.all_nodes[tuple(cand_pt)]['pred'] = \
                            current_node['pos']
                        break

                    if cand_pt == self.goal_pt:
                        current_node['open'] = False
                        self.all_nodes[tuple(cand_pt)]['pred'] = \
                            current_node['pos']
                        goal_found = True
                        break

                    cand_pt = tuple(cand_pt)
                    no_valid_f = self.update_node_cost(cand_pt, curr_f_thresh,
                                                       current_node,
                                                       f_cost_list, no_valid_f,
                                                       offset, w)
                if goal_found:
                    break
            if show_animation:
                plt.pause(0.001)
            if goal_found:
                current_node = self.all_nodes[tuple(self.goal_pt)]
            while goal_found:
                if current_node['pred'] is None:
                    break
                if use_theta_star or use_jump_point:
                    x, y = [current_node['pos'][0], current_node['pred'][0]], \
                             [current_node['pos'][1], current_node['pred'][1]]
                    if show_animation:
                        plt.plot(x, y, "b")
                else:
                    if show_animation:
                        plt.plot(current_node['pred'][0],
                                 current_node['pred'][1], "b*")
                current_node = self.all_nodes[tuple(current_node['pred'])]
            if goal_found:
                break

            if use_iterative_deepening and f_cost_list:
                curr_f_thresh = min(f_cost_list)
            if use_iterative_deepening and not f_cost_list:
                curr_f_thresh = np.inf
            if use_iterative_deepening and not f_cost_list and no_valid_f:
                current_node['fcost'], current_node['hcost'] = np.inf, np.inf
                continue

            current_node['open'] = False
            current_node['in_open_list'] = False
            if show_animation:
                plt.plot(current_node['pos'][0], current_node['pos'][1], "g*")
            del self.open_set[p]
            current_node['fcost'], current_node['hcost'] = np.inf, np.inf
            depth += 1
        if show_animation:
            plt.show()

    def update_node_cost(self, cand_pt, curr_f_thresh, current_node,
                         f_cost_list, no_valid_f, offset, w):
        if not self.obs_grid[tuple(cand_pt)] and \
                self.all_nodes[cand_pt]['open']:
            g_cost = offset + current_node['gcost']
            h_cost = self.all_nodes[cand_pt]['hcost']
            if use_dynamic_weighting:
                h_cost = h_cost * w
            f_cost = g_cost + h_cost
            if f_cost < self.all_nodes[cand_pt]['fcost'] and \
                    f_cost <= curr_f_thresh:
                f_cost_list.append(f_cost)
                self.all_nodes[cand_pt]['pred'] = \
                    current_node['pos']
                self.all_nodes[cand_pt]['gcost'] = g_cost
                self.all_nodes[cand_pt]['fcost'] = f_cost
                if not self.all_nodes[cand_pt]['in_open_list']:
                    self.open_set.append(self.all_nodes[cand_pt])
                    self.all_nodes[cand_pt]['in_open_list'] = True
                if show_animation:
                    plt.plot(cand_pt[0], cand_pt[1], "r*")
            if curr_f_thresh < f_cost < \
                    self.all_nodes[cand_pt]['fcost']:
                no_valid_f = True
        return no_valid_f


def main():
    # set obstacle positions
    obs_dict = {}
    for i in range(51):
        for j in range(51):
            obs_dict[(i, j)] = False
    o_x, o_y = [], []

    s_x = 5.0
    s_y = 5.0
    g_x = 35.0
    g_y = 45.0

    # draw outer border of maze
    draw_vertical_line(0, 0, 50, o_x, o_y, obs_dict)
    draw_vertical_line(48, 0, 50, o_x, o_y, obs_dict)
    draw_horizontal_line(0, 0, 50, o_x, o_y, obs_dict)
    draw_horizontal_line(0, 48, 50, o_x, o_y, obs_dict)

    # draw inner walls
    all_x = [10, 10, 10, 15, 20, 20, 30, 30, 35, 30, 40, 45]
    all_y = [10, 30, 45, 20, 5, 40, 10, 40, 5, 40, 10, 25]
    all_len = [10, 10, 5, 10, 10, 5, 20, 10, 25, 10, 35, 15]
    for x, y, l in zip(all_x, all_y, all_len):
        draw_vertical_line(x, y, l, o_x, o_y, obs_dict)

    all_x[:], all_y[:], all_len[:] = [], [], []
    all_x = [35, 40, 15, 10, 45, 20, 10, 15, 25, 45, 10, 30, 10, 40]
    all_y = [5, 10, 15, 20, 20, 25, 30, 35, 35, 35, 40, 40, 45, 45]
    all_len = [10, 5, 10, 10, 5, 5, 10, 5, 10, 5, 10, 5, 5, 5]
    for x, y, l in zip(all_x, all_y, all_len):
        draw_horizontal_line(x, y, l, o_x, o_y, obs_dict)

    if show_animation:
        plt.plot(o_x, o_y, ".k")
        plt.plot(s_x, s_y, "og")
        plt.plot(g_x, g_y, "xb")
        plt.grid(True)

    if use_jump_point:
        keypoint_list = key_points(obs_dict)
        search_obj = SearchAlgo(obs_dict, g_x, g_y, s_x, s_y, 101, 101,
                                keypoint_list)
        search_obj.jump_point()
    else:
        search_obj = SearchAlgo(obs_dict, g_x, g_y, s_x, s_y, 101, 101)
        search_obj.a_star()


if __name__ == '__main__':
    main()
