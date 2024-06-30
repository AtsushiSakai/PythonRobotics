import heapq
import math

class AStar:
    class Node:
        def __init__(self, x, y, cost, parent):
            self.x = x
            self.y = y
            self.cost = cost
            self.parent = parent
            self.f_cost = 0.0  # Total cost (g_cost + h_cost)

        def __lt__(self, other):
            return self.f_cost < other.f_cost

    def __init__(self, start, goal, obstacle_list, grid_size, robot_radius):
        self.start = self.Node(start[0], start[1], 0.0, None)
        self.goal = self.Node(goal[0], goal[1], 0.0, None)
        self.obstacle_list = obstacle_list
        self.grid_size = grid_size
        self.robot_radius = robot_radius
        self.motion = self.get_motion_model()

    def planning(self):
        open_list = []
        closed_list = set()
        heapq.heappush(open_list, self.start)

        while open_list:
            current = heapq.heappop(open_list)
            closed_list.add((current.x, current.y))

            if self.is_goal(current):
                return self.extract_path(current)

            for move_x, move_y, move_cost in self.motion:
                node = self.Node(current.x + move_x, current.y + move_y, current.cost + move_cost, current)
                node.f_cost = node.cost + self.heuristic(node, self.goal)

                if (node.x, node.y) in closed_list or not self.is_free(node):
                    continue

                heapq.heappush(open_list, node)

        return None

    def extract_path(self, node):
        path = [[node.x, node.y]]
        while node.parent is not None:
            node = node.parent
            path.append([node.x, node.y])
        path.reverse()
        return path

    def is_goal(self, node):
        return self.calc_distance(node, self.goal) <= self.grid_size

    def is_free(self, node):
        for (ox, oy, size) in self.obstacle_list:
            if math.hypot(ox - node.x, oy - node.y) <= size + self.robot_radius:
                return False
        return True

    def calc_distance(self, node1, node2):
        return math.hypot(node2.x - node1.x, node2.y - node1.y)

    def heuristic(self, node1, node2):
        return self.calc_distance(node1, node2)

    @staticmethod
    def get_motion_model():
        # dx, dy, cost
        return [
            (1, 0, 1), (0, 1, 1), (-1, 0, 1), (0, -1, 1),
            (-1, -1, math.sqrt(2)), (-1, 1, math.sqrt(2)),
            (1, -1, math.sqrt(2)), (1, 1, math.sqrt(2))
        ]

