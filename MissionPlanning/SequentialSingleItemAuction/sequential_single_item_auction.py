"""

Multi-robot task allocation with a Sequential Single-Item (SSI) auction

author: Vishwa (@vishwagw)

A team of robots (e.g. drones) must visit a set of target locations. The
auction assigns targets one at a time: in every round, each robot bids on each
unassigned target, and the target with the single lowest bid goes to the robot
that made it. A robot's bid is the extra travel cost of inserting the target
into its current route at the best position. Travel costs are A* path lengths
on an occupancy grid, so obstacles are taken into account.

Two team objectives are supported:

- MiniSum: minimize the total distance travelled by all robots.
  bid = increase in the bidding robot's route cost
- MiniMax: minimize the longest route (the mission's completion time).
  bid = the bidding robot's route cost after adding the target

References:

- M. G. Lagoudakis, E. Markakis, D. Kempe, P. Keskinocak, A. Kleywegt,
  S. Koenig, C. Tovey, A. Meyerson, S. Jain,
  "Auction-Based Multi-Robot Routing", Robotics: Science and Systems, 2005.
  https://www.roboticsproceedings.org/rss01/p45.pdf
- S. Koenig, C. Tovey, M. Lagoudakis, V. Markakis, D. Kempe, P. Keskinocak,
  A. Kleywegt, A. Meyerson, S. Jain,
  "The Power of Sequential Single-Item Auctions for Agent Coordination",
  AAAI, 2006.

"""
import contextlib
import io
import math
import pathlib
import sys

import matplotlib.pyplot as plt

sys.path.append(str(pathlib.Path(__file__).parent.parent.parent))

from PathPlanning.AStar import a_star  # noqa: E402

show_animation = True

MINISUM = "minisum"
MINIMAX = "minimax"


class TravelCostMap:
    """
    Obstacle-aware travel costs between points, computed with A* and cached.

    The A* grid only contains discrete cells, so every point is snapped to its
    nearest grid cell before planning.
    """

    def __init__(self, obstacle_x, obstacle_y, grid_resolution, robot_radius):
        """
        obstacle_x: x position list of obstacles [m]
        obstacle_y: y position list of obstacles [m]
        grid_resolution: grid cell size [m]
        robot_radius: robot radius used to inflate obstacles [m]
        """
        a_star.show_animation = False
        with contextlib.redirect_stdout(io.StringIO()):  # silence A* logs
            self.planner = a_star.AStarPlanner(obstacle_x, obstacle_y,
                                               grid_resolution, robot_radius)
        self.paths = {}  # (start, goal) -> (path_x, path_y, length)

    def path(self, start, goal):
        """
        Shortest path from start to goal.

        start: (x, y) start position [m]
        goal: (x, y) goal position [m]
        returns: (path_x, path_y, length) with positions and length in [m]
        """
        key = (start, goal)
        if key not in self.paths:
            with contextlib.redirect_stdout(io.StringIO()):  # silence A* logs
                path_x, path_y = self.planner.planning(start[0], start[1],
                                                       goal[0], goal[1])
            path_x, path_y = path_x[::-1], path_y[::-1]  # A* returns goal first
            length = sum(math.hypot(x1 - x0, y1 - y0) for x0, y0, x1, y1 in
                         zip(path_x, path_y, path_x[1:], path_y[1:]))
            self.paths[key] = (path_x, path_y, length)
            self.paths[(goal, start)] = (path_x[::-1], path_y[::-1], length)
        return self.paths[key]

    def cost(self, start, goal):
        """Travel distance from start to goal [m]."""
        return self.path(start, goal)[2]


def route_cost(cost_map, start, targets):
    """
    Length of an open route that starts at `start` and visits `targets` in order.

    start: (x, y) robot start position [m]
    targets: list of (x, y) target positions [m]
    returns: route length [m]
    """
    waypoints = [start] + list(targets)
    return sum(cost_map.cost(a, b) for a, b in zip(waypoints, waypoints[1:]))


def best_insertion(cost_map, start, route, target):
    """
    Cheapest position to insert `target` into a robot's ordered route.

    Every insertion position is tried while the order of the targets already
    in the route is kept.

    Parameters
    ----------
    cost_map : TravelCostMap
        travel costs between points
    start : tuple
        (x, y) robot start position [m]
    route : list
        ordered (x, y) targets the robot already visits [m]
    target : tuple
        (x, y) target to insert [m]

    Returns
    -------
    insert_index : int
        position in `route` where the target is inserted
    new_route_cost : float
        route cost after the insertion [m]
    """
    best_index, best_cost = None, math.inf
    for insert_index in range(len(route) + 1):
        candidate = route[:insert_index] + [target] + route[insert_index:]
        cost = route_cost(cost_map, start, candidate)
        if cost < best_cost:
            best_index, best_cost = insert_index, cost
    return best_index, best_cost


def calc_bid(objective, current_cost, new_cost):
    """
    A robot's bid for a target under the chosen team objective.

    MiniSum bids the marginal cost (how much the team's total grows), while
    MiniMax bids the robot's resulting route cost (how long this robot will
    now be busy).

    Parameters
    ----------
    objective : str
        MINISUM or MINIMAX
    current_cost : float
        robot's route cost before adding the target [m]
    new_cost : float
        robot's route cost after adding the target [m]

    Returns
    -------
    bid : float
        the robot's bid; the lowest bid wins [m]
    """
    if objective == MINISUM:
        return new_cost - current_cost
    if objective == MINIMAX:
        return new_cost
    raise ValueError(f"Unknown objective: {objective}")


def sequential_single_item_auction(robot_starts, targets, cost_map,
                                   objective=MINISUM, round_callback=None):
    """
    Allocate every target to exactly one robot with an SSI auction.

    Parameters
    ----------
    robot_starts : list
        (x, y) robot start positions [m]
    targets : list
        (x, y) target positions [m]
    cost_map : TravelCostMap
        travel costs between points
    objective : str, optional
        MINISUM (default) or MINIMAX
    round_callback : callable, optional
        called after every round as
        ``round_callback(routes, winner_index, won_target)``,
        e.g. for visualization

    Returns
    -------
    routes : list
        one ordered list of (x, y) targets per robot
    """
    routes = [[] for _ in robot_starts]
    route_costs = [0.0 for _ in robot_starts]
    unallocated = list(targets)

    while unallocated:
        winning_bid = None  # (bid, robot_index, target, insert_index, new_cost)
        for robot_index, start in enumerate(robot_starts):
            for target in unallocated:
                insert_index, new_cost = best_insertion(
                    cost_map, start, routes[robot_index], target)
                bid = calc_bid(objective, route_costs[robot_index], new_cost)
                if winning_bid is None or bid < winning_bid[0]:
                    winning_bid = (bid, robot_index, target, insert_index,
                                   new_cost)

        _, winner, target, insert_index, new_cost = winning_bid
        routes[winner].insert(insert_index, target)
        route_costs[winner] = new_cost
        unallocated.remove(target)

        if round_callback is not None:
            round_callback(routes, winner, target)

    return routes


def build_map(size, walls):
    """
    Obstacle points for a square area with a border and interior walls.

    size: side length of the square area [m]
    walls: list of ((x0, y0), (x1, y1)) axis-aligned wall segments [m]
    returns: (obstacle_x, obstacle_y) lists [m]
    """
    obstacle_x, obstacle_y = [], []
    border = [((0, 0), (size, 0)), ((size, 0), (size, size)),
              ((0, size), (size, size)), ((0, 0), (0, size))]
    for (x0, y0), (x1, y1) in border + walls:
        steps = int(max(abs(x1 - x0), abs(y1 - y0)))
        for i in range(steps + 1):
            obstacle_x.append(x0 + (x1 - x0) * i / steps)
            obstacle_y.append(y0 + (y1 - y0) * i / steps)
    return obstacle_x, obstacle_y


ROBOT_COLORS = ["tab:blue", "tab:orange", "tab:green", "tab:purple"]


def plot_state(cost_map, obstacles, robot_starts, targets, routes, title):
    """Draw obstacles, robots, targets and every robot's current route."""
    plt.cla()
    plt.plot(obstacles[0], obstacles[1], ".k", markersize=2)
    assigned = {target for route in routes for target in route}
    unassigned = [t for t in targets if t not in assigned]
    plt.plot([t[0] for t in unassigned], [t[1] for t in unassigned], "x",
             color="gray", markersize=8, label="unassigned target")
    for robot_index, (start, route) in enumerate(zip(robot_starts, routes)):
        color = ROBOT_COLORS[robot_index % len(ROBOT_COLORS)]
        waypoints = [start] + route
        for a, b in zip(waypoints, waypoints[1:]):
            path_x, path_y, _ = cost_map.path(a, b)
            plt.plot(path_x, path_y, "-", color=color, linewidth=2)
        plt.plot([t[0] for t in route], [t[1] for t in route], "o",
                 color=color, markersize=7)
        plt.plot(start[0], start[1], "^", color=color, markersize=12,
                 label=f"robot {robot_index}")
    plt.title(title, fontsize=10)
    plt.axis("equal")
    plt.legend(loc="upper left", bbox_to_anchor=(1.01, 1.0), fontsize=8)
    plt.tight_layout()


def main(objective=MINISUM):
    print(__file__ + " start!!")

    # simulation parameters
    area_size = 60.0  # [m]
    grid_resolution = 2.0  # [m]
    robot_radius = 1.0  # [m]
    walls = [((20, 0), (20, 35)), ((40, 25), (40, 60))]
    robot_starts = [(5.0, 5.0), (55.0, 5.0), (30.0, 55.0)]
    targets = [(42.0, 8.0), (34.0, 20.0), (46.0, 16.0), (36.0, 36.0),
               (48.0, 14.0), (46.0, 36.0), (10.0, 52.0), (48.0, 24.0),
               (40.0, 14.0), (28.0, 18.0)]

    obstacles = build_map(area_size, walls)
    cost_map = TravelCostMap(obstacles[0], obstacles[1], grid_resolution,
                             robot_radius)

    round_number = [0]

    def show_round(routes, winner, target):
        round_number[0] += 1
        if show_animation:  # pragma: no cover
            plot_state(cost_map, obstacles, robot_starts, targets, routes,
                       f"{objective} auction, round {round_number[0]}: "
                       f"robot {winner} wins {target}")
            # for stopping simulation with the esc key.
            plt.gcf().canvas.mpl_connect(
                "key_release_event",
                lambda event: [exit(0) if event.key == "escape" else None])
            plt.pause(0.5)

    routes = sequential_single_item_auction(robot_starts, targets, cost_map,
                                            objective, show_round)

    costs = [route_cost(cost_map, s, r) for s, r in zip(robot_starts, routes)]
    for robot_index, (route, cost) in enumerate(zip(routes, costs)):
        print(f"robot {robot_index}: {len(route)} targets, {cost:.1f} m")
    print(f"total distance: {sum(costs):.1f} m, longest route: {max(costs):.1f} m")

    if show_animation:  # pragma: no cover
        plot_state(cost_map, obstacles, robot_starts, targets, routes,
                   f"{objective}: total {sum(costs):.0f} m, "
                   f"longest {max(costs):.0f} m")
        plt.show()

    return routes, costs


if __name__ == '__main__':
    main(MINISUM)
    main(MINIMAX)
