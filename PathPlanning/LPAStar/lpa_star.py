import math
import time
from sys import maxsize
import matplotlib.pyplot as plt
show_animation = True
show_process = False
max_iteration = 0

class State:

    def __init__(self, x, y):
        self.x = x
        self.y = y
        self.parent = None
        self.iteration = 0
        self.state = "."  # tag for state
        self.g = maxsize
        self.rhs = maxsize
        self.h = 0
        self.k = []
        self.black_list = []

    def cost(self, state):
        if self.state == "#" or state.state == "#":
            return maxsize

        return math.sqrt(math.pow((self.x - state.x), 2) +
                         math.pow((self.y - state.y), 2))

    def calculate_key(self):
        self.k = [min(self.g, self.rhs) + self.h, min(self.g, self.rhs)]
        return self.k

    def set_state(self, state):
        """
        .: new
        #: obstacle
        e: oparent of current state
        *: closed state
        s: current state
        """
        if state not in ["s", ".", "#", "e", "*"]:
            return
        self.state = state
    
    def init_state(self):
        if self.iteration != max_iteration:
                        self.g = maxsize
                        self.rhs = maxsize
                        self.parent = None
                        self.iteration = max_iteration


class Map:

    def __init__(self, row, col):
        self.row = row
        self.col = col
        self.map = self.init_map()

    def init_map(self):
        map_list = []
        for i in range(self.row):
            tmp = []
            for j in range(self.col):
                tmp.append(State(i, j))
            map_list.append(tmp)
        return map_list

    def init_h(self, goal):
        for i in self.map:
            for j in i:
                j.h = j.cost(goal)

    def get_neighbors(self, state):
        state_list = []
        for i in [-1, 0, 1]:
            for j in [-1, 0, 1]:
                if i == 0 and j == 0:
                    continue
                if state.x + i < 0 or state.x + i >= self.row:
                    continue
                if state.y + j < 0 or state.y + j >= self.col:
                    continue
                if self.map[state.x + i][state.y + j] in state.black_list:
                    continue
                state_list.append(self.map[state.x + i][state.y + j])
        return state_list

    def set_obstacle(self, point_list):
        for x, y in point_list:
            if x < 0 or x >= self.row or y < 0 or y >= self.col:
                continue

            self.map[x][y].set_state("#")

class LPA:
    def __init__(self, maps, start, goal, add_obstacle_first):
        global max_iteration
        max_iteration += 1
        self.map = maps
        self.open_list = set()
        self.start = start
        self.goal = goal
        self.start.iteration = max_iteration
        self.goal.iteration = max_iteration
        self.start.rhs = 0
        self.map.init_h(goal)
        self.insert(self.start)
        self.add_obstacle_first = add_obstacle_first
        self.count_expend = 0

    def run(self):
        if self.add_obstacle_first:
            self.AddNewObstacle()
        start_time = time.time()
        self.compute_shortest_path()
        end_time = time.time()
        print("first_search: ", end_time - start_time, self.count_expend)

        if not self.add_obstacle_first:
            self.AddNewObstacle()
        
        global show_process
        show_process = False
        self.count_expend = 0
        start_time = time.time()
        self.compute_shortest_path()
        end_time = time.time()
        print("second_search ( search after the change): ", end_time - start_time, self.count_expend)
        
        
        self.plot_path()
    
    def compute_shortest_path(self):
        while self.goal.rhs > self.goal.g or self.compare_key(self.get_kmin(), self.goal.calculate_key()):
            tmp = self.get_min_state()
            self.count_expend += 1
            if tmp.g > tmp.rhs:
                tmp.g = tmp.rhs
                self.remove(tmp)
                for y in self.map.get_neighbors(tmp):
                    y.init_state()
                    if y.rhs > tmp.g + y.cost(tmp) and y != self.start:
                        y.parent = tmp
                        y.rhs = tmp.g + y.cost(tmp)
                        self.UpdateVertex(y)
                    if show_animation and show_process:
                        # plt.pause(0.001)
                        plt.plot(y.x, y.y, 'xg')
            else:
                tmp.g = maxsize
                self.UpdateVertex(tmp)
                for y in self.map.get_neighbors(tmp):
                    y.init_state()
                    if y != self.start and y.parent == tmp:
                        self.update_rhs(y)
                    if show_animation and show_process:
                        # plt.pause(0.001)
                        plt.plot(y.x, y.y, 'xg')

    def AddNewObstacle(self):
        ox, oy = [], []
        for i in range(10, 21):
            ox.append(i)
            oy.append(12)
        # for i in range(41, 51):
        #     ox.append(5)
        #     oy.append(i)
        # for i in range(5, 21):
        #     ox.append(i)
        #     oy.append(50)
        self.map.set_obstacle([(i, j) for i, j in zip(ox, oy)])
        if show_animation:
            plt.pause(0.001)
            plt.plot(ox, oy, ".g")
        
        for i, j in zip(ox, oy):
            change = self.map.map[i][j]
            change.init_state()
            for y in self.map.get_neighbors(change):
                if y.state != "#":
                    change.black_list.append(y)
                    y.black_list.append(change)
                    y.init_state()
                    if y.parent == change and y != self.start:
                        self.update_rhs(y)
            if change != self.start:
                change.rhs = maxsize
                self.UpdateVertex(change)

    def plot_path(self):
        tmp = self.goal
        rx, ry = [], []
        rx.append(tmp.x)
        ry.append(tmp.y)
        while tmp != self.start:
            tmp = tmp.parent
            rx.append(tmp.x)
            ry.append(tmp.y)
            if show_animation:
                plt.pause(0.001)
                plt.plot(rx, ry, '-r')

    def update_rhs(self, state):
        state.rhs = maxsize
        state.parent = None
        for y in self.map.get_neighbors(state):
            if y.iteration == max_iteration and state.rhs > y.g + y.cost(state):
                state.rhs = y.g + y.cost(state)
                state.parent = y
        self.UpdateVertex(state)

    def UpdateVertex(self, state):
        # if state != self.start:
        #     state.rhs = min([y.g + state.cost(y) for y in self.map.get_neighbors(state)])
        # if state in self.open_list:
        #     self.remove(state)
        # if state.g != state.rhs:
        #     self.insert(state)
        if state.g != state.rhs:
            self.insert(state)
        else:
            if state in self.open_list:
                self.remove(state)
    

    def compare_key(self, a, b):
        if a[0] < b[0]:
            return True
        if a[0] == b[0] and a[1] < b[1]:
            return True
        return False

    def insert(self, state):
        state.calculate_key()
        self.open_list.add(state)
    
    def remove(self, state):
        self.open_list.remove(state)

    def get_kmin(self):
        min_state = min(self.open_list, key=lambda x: (x.k[0], x.k[1]))
        return min_state.k

    def get_min_state(self):
        min_state = min(self.open_list, key=lambda x: (x.k[0], x.k[1]))
        return min_state
    

def main():
    start = [10, 10]
    goal = [50, 50]
    m = Map(60, 60)
    ox, oy = [], []
    for i in range(0, 60):
        ox.append(i)
        oy.append(0)
    for i in range(0, 60):
        ox.append(60)
        oy.append(i)
    for i in range(0, 61):
        ox.append(i)
        oy.append(60)
    for i in range(0, 61):
        ox.append(0)
        oy.append(i)
    for i in range(0, 40):
        ox.append(20)
        oy.append(i)
    for i in range(0, 40):
        ox.append(40)
        oy.append(60 - i)
    m.set_obstacle([(i, j) for i, j in zip(ox, oy)])

    if show_animation:
        plt.plot(ox, oy, ".k")
        plt.plot(start[0], start[1], "og")
        plt.plot(goal[0], goal[1], "xb")
        plt.axis("equal")

    start = m.map[start[0]][start[1]]
    end = m.map[goal[0]][goal[1]]
    lpa = LPA(m, start, end, False)
    lpa.run()


    if show_animation:
        plt.show()
    

if __name__ == '__main__':
    main()