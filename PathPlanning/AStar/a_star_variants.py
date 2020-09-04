"""
astar variants
author: Sarim Mehdi(muhammadsarim.mehdi@studio.unibo.it)
Source: http://theory.stanford.edu/~amitp/GameProgramming/Variations.html
"""

import time
import numpy as np
import matplotlib.pyplot as plt

show_animation = True
use_beam_search = False
use_iterative_deepening = False
use_dynamic_weighting = True


def draw_horizontal_line(start_x, start_y, length, o_x, o_y, o_dict):
    for i in range(start_x, start_x + length):
        for j in range(start_y, start_y + 1):
            o_x.append(i)
            o_y.append(j)
            o_dict[(i, j)] = True


def draw_vertical_line(start_x, start_y, length, o_x, o_y, o_dict):
    for i in range(start_x, start_x + 1):
        for j in range(start_y, start_y + length):
            o_x.append(i)
            o_y.append(j)
            o_dict[(i, j)] = True


class Search_Algo:
    def __init__(self, obs_grid, goal_x, goal_y, start_x, start_y,
                 limit_x, limit_y):
        self.start_pt = [start_x, start_y]
        self.goal_pt = [goal_x, goal_y]
        self.obs_grid = obs_grid
        g_cost, h_cost = 0, self.get_hval(start_x, start_y, goal_x, goal_y)
        f_cost = g_cost + h_cost
        self.all_nodes, self.open_set = {}, []
        for i in range(limit_x):
            for j in range(limit_y):
                h_c = self.get_hval(i, j, goal_x, goal_y)
                self.all_nodes[(i, j)] = {'pos': [i, j], 'pred': None,
                                          'gcost': np.inf, 'hcost': h_c,
                                          'fcost': np.inf,
                                          'open': True, 'in_open_list': False}
        self.all_nodes[tuple(self.start_pt)] = \
            {'pos': self.start_pt, 'pred': None,
             'gcost': g_cost, 'hcost': h_cost, 'fcost': f_cost,
             'open': True, 'in_open_list': True}
        self.open_set.append(self.all_nodes[tuple(self.start_pt)])

    def get_hval(self, x1, y1, x2, y2):
        x, y = x1, y1
        val = 0
        while x != x2 or y != y2:
            if x != x2 and y != y2:
                val += 14
            else:
                val += 10
            x, y = x + np.sign(x2 - x), y + np.sign(y2 - y)
        return val

    def astar(self):
        '''Beam search: Maintain an open list of just 30 nodes.
        If more than 30 nodes, then get rid of ndoes with high
        f values.
        Iterative deepening: At every iteration, get a cut-off
        value for the f cost. This cut-off is minimum of the f
        value of all nodes whose f value is higher than the
        current cut-off value. Nodes with f value higher than
        the current cut off value are not put in the open set.
        Dynamic weighting: Multiply heuristic with the following:
        (1 + epsilon - (epsilon*d)/N) where d is the current
        iteration of loop and N is upper bound on number of
        iterations'''
        plt.title('A*')

        goal_found = False
        curr_f_thresh = np.inf
        depth, upper_bound_depth = 0, 500
        while len(self.open_set) > 0:
            self.open_set = sorted(self.open_set, key=lambda x: x['fcost'])
            lowest_f = self.open_set[0]['fcost']
            lowest_h = self.open_set[0]['hcost']
            lowest_g = self.open_set[0]['gcost']
            p = 0
            for p_n in self.open_set[1:]:
                if p_n['fcost'] == lowest_f and \
                        p_n['hcost'] < lowest_h:
                    lowest_h = p_n['hcost']
                    p += 1
                elif p_n['fcost'] == lowest_f and \
                        p_n['hcost'] == lowest_h and \
                        p_n['gcost'] < lowest_g:
                    lowest_g = p_n['gcost']
                    p += 1
                else:
                    break
            current_node = self.all_nodes[tuple(self.open_set[p]['pos'])]

            while len(self.open_set) > 10 and use_beam_search:
                del self.open_set[-1]

            f_cost_list = []
            alternate_f_cost_list = []
            w, epsilon = 1, 4
            if use_dynamic_weighting:
                w = (1 + epsilon - epsilon*depth/upper_bound_depth)
            for i in [-1, 0, 1]:
                for j in [-1, 0, 1]:
                    if i == 0 and j == 0:
                        continue
                    offset = 10 if min(i, j) == 0 else 14
                    cand_pt = [current_node['pos'][0] + i,
                               current_node['pos'][1] + j]

                    if cand_pt == self.goal_pt:
                        current_node['open'] = False
                        self.all_nodes[tuple(cand_pt)]['pred'] = \
                            current_node['pos']
                        goal_found = True
                        break

                    cand_pt = tuple(cand_pt)
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
                            plt.plot(cand_pt[0], cand_pt[1], "r*")
                        alternate_f = min(f_cost,
                                          self.all_nodes[cand_pt]['fcost'])
                        if alternate_f > curr_f_thresh:
                            alternate_f_cost_list.append(alternate_f)
                if goal_found:
                    break
            plt.pause(0.001)
            if goal_found:
                current_node = self.all_nodes[tuple(self.goal_pt)]
            while goal_found:
                if current_node['pred'] is None:
                    break
                plt.plot(current_node['pred'][0],
                         current_node['pred'][1], "b*")
                current_node = self.all_nodes[tuple(current_node['pred'])]
                plt.pause(0.001)
            if goal_found:
                break

            if use_iterative_deepening and f_cost_list and \
                    not alternate_f_cost_list:
                curr_f_thresh = min(f_cost_list)
            if use_iterative_deepening and not f_cost_list and \
                    alternate_f_cost_list:
                curr_f_thresh = min(alternate_f_cost_list)
            if use_iterative_deepening and not f_cost_list and \
                    not alternate_f_cost_list:
                curr_f_thresh = np.inf

            current_node['open'] = False
            current_node['in_open_list'] = False
            plt.plot(current_node['pos'][0], current_node['pos'][1], "g*")
            del self.open_set[p]
            current_node['fcost'], current_node['hcost'] = np.inf, np.inf
            depth += 1
        if show_animation:
            plt.show()


def main():
    # set obstacle positions
    obs_dict = {}
    for i in range(101):
        for j in range(101):
            obs_dict[(i, j)] = False
    o_x, o_y = [], []

    s_x = 10.0
    s_y = 10.0
    g_x = 90.0
    g_y = 90.0

    # draw outer border of maze
    draw_vertical_line(0, 0, 100, o_x, o_y, obs_dict)
    draw_vertical_line(98, 0, 100, o_x, o_y, obs_dict)
    draw_horizontal_line(0, 0, 100, o_x, o_y, obs_dict)
    draw_horizontal_line(0, 98, 100, o_x, o_y, obs_dict)

    # draw inner walls
    all_x = [25, 25, 50, 50, 70, 85, 12, 35, 35]
    all_y = [25, 30, 0, 75, 50, 0, 35, 70, 0]
    all_len = [50, 40, 25, 25, 25, 40, 30, 20, 30]
    for x, y, l in zip(all_x, all_y, all_len):
        draw_vertical_line(x, y, l, o_x, o_y, obs_dict)

    all_x[:], all_y[:], all_len[:] = [], [], []
    all_x = [50, 25, 0, 25, 40, 80, 65, 90]
    all_y = [25, 50, 80, 50, 60, 60, 10, 70]
    all_len = [25, 50, 10, 10, 20, 20, 10, 10]
    for x, y, l in zip(all_x, all_y, all_len):
        draw_horizontal_line(x, y, l, o_x, o_y, obs_dict)

    plt.plot(o_x, o_y, ".k")
    plt.plot(s_x, s_y, "og")
    plt.plot(g_x, g_y, "xb")
    plt.grid(True)

    search_obj = Search_Algo(obs_dict, g_x, g_y, s_x, s_y, 101, 101)
    search_obj.astar()


if __name__ == '__main__':
    main()
