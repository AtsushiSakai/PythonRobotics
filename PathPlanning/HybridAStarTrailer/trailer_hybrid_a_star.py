"""Hybrid A* for a car with one passive, on-axle trailer.

Author: Atsushi Sakai (with Codex)
References:
- Dolgov et al., Practical Search Techniques in Path Planning for Autonomous Driving
- LaValle, Planning Algorithms, Section 13.1.2.4
- Atsushi Sakai's HybridAStarTrailer (Julia), https://github.com/yinflight/HybridAStarTrailer

Positions and lengths are in metres; headings and steering angles are in radians.
The hitch is at the tractor rear axle. Obstacles are sampled points.
"""

from __future__ import annotations

from dataclasses import dataclass
import heapq
import itertools
import math
import pathlib
import sys

import matplotlib.pyplot as plt
import numpy as np
from scipy.spatial import cKDTree

sys.path.append(str(pathlib.Path(__file__).resolve().parents[2]))
from PathPlanning.ReedsSheppPath import reeds_shepp_path_planning as rs

# Geometry from the reference trailer example, relative to the hitch.
WHEEL_BASE = 3.7
TRAILER_LENGTH = 8.0  # hitch to trailer axle
WIDTH = 2.6
TRACTOR_FRONT, TRACTOR_REAR = 4.5, 1.0
TRAILER_FRONT, TRAILER_REAR = 1.0, 9.0
MAX_STEER = 0.6
MAX_ARTICULATION = math.radians(75.0)

XY_RESOLUTION = 2.0
YAW_RESOLUTION = math.radians(15.0)
MOTION_RESOLUTION = 0.2
PRIMITIVE_LENGTH = 3.0
N_STEER = 7
TRAILER_GOAL_TOLERANCE = math.radians(5.0)
SEARCH_MARGIN = 15.0
BACK_COST = 5.0
SWITCH_COST = 10.0
STEER_COST = 1.0
STEER_CHANGE_COST = 1.0
ARTICULATION_COST = 1.0
HEURISTIC_WEIGHT = 3.0
show_animation = True


@dataclass
class Node:
    """A continuous trajectory segment and its predecessor in the search."""

    poses: list
    distances: list
    steer: float = 0.0
    direction: int = 0
    cost: float = 0.0
    parent: Node | None = None


@dataclass
class Path:
    """Sampled (x, y, tractor yaw, trailer yaw), signed steps, and search cost.

    ``distances[i]`` and ``steers[i]`` produce ``poses[i]`` from the previous
    pose. Their first entries are zero. ``expanded_nodes`` counts heap pops.
    """

    poses: np.ndarray
    distances: np.ndarray
    steers: np.ndarray
    cost: float
    expanded_nodes: int


def wrap_angle(angle):
    """Map an angle or an array of angles to [-pi, pi)."""
    return (angle + math.pi) % (2.0 * math.pi) - math.pi


def move(pose, distance, steer):
    """Integrate a constant-steering signed step, including reverse motion.

    The tractor follows an exact circular arc. The passive trailer heading
    follows d(phi)/ds = sin(theta - phi) / TRAILER_LENGTH, integrated with RK4.
    """
    x, y, yaw, trailer_yaw = pose
    curvature = math.tan(steer) / WHEEL_BASE
    yaw_change = distance * curvature
    # sinc avoids division by zero for a straight segment.
    travel = distance * np.sinc(yaw_change / (2.0 * math.pi))
    x += travel * math.cos(yaw + yaw_change / 2.0)
    y += travel * math.sin(yaw + yaw_change / 2.0)

    def derivative(offset, heading):
        return math.sin(yaw + curvature * offset - heading) / TRAILER_LENGTH

    k1 = derivative(0.0, trailer_yaw)
    k2 = derivative(distance / 2.0, trailer_yaw + distance * k1 / 2.0)
    k3 = derivative(distance / 2.0, trailer_yaw + distance * k2 / 2.0)
    k4 = derivative(distance, trailer_yaw + distance * k3)
    trailer_yaw += distance * (k1 + 2.0 * k2 + 2.0 * k3 + k4) / 6.0
    return x, y, wrap_angle(yaw + yaw_change), wrap_angle(trailer_yaw)


def collision_free(poses, obstacle_tree):
    """Check both rectangular bodies at every pose against obstacle points."""
    poses = np.asarray(poses)
    for heading_column, front, rear in (
        (2, TRACTOR_FRONT, TRACTOR_REAR),
        (3, TRAILER_FRONT, TRAILER_REAR),
    ):
        headings = poses[:, heading_column]
        offsets = (front - rear) / 2.0
        centers = poses[:, :2] + offsets * np.column_stack(
            (np.cos(headings), np.sin(headings)))
        radius = math.hypot((front + rear) / 2.0, WIDTH / 2.0)
        neighbors = obstacle_tree.query_ball_point(centers, radius)
        for pose, heading, indices in zip(poses, headings, neighbors):
            if not indices:
                continue
            delta = obstacle_tree.data[indices] - pose[:2]
            local_x = delta[:, 0] * math.cos(heading) + delta[:, 1] * math.sin(heading)
            local_y = -delta[:, 0] * math.sin(heading) + delta[:, 1] * math.cos(heading)
            if np.any((local_x >= -rear) & (local_x <= front)
                      & (np.abs(local_y) <= WIDTH / 2.0)):
                return False
    return True


def valid_poses(poses, obstacle_tree, bounds):
    """Reject out-of-bounds hitch positions, excessive articulation, or collision."""
    poses = np.asarray(poses)
    xmin, xmax, ymin, ymax = bounds
    return (np.all((poses[:, 0] >= xmin) & (poses[:, 0] <= xmax)
                   & (poses[:, 1] >= ymin) & (poses[:, 1] <= ymax))
            and np.all(np.abs(wrap_angle(poses[:, 2] - poses[:, 3]))
                       <= MAX_ARTICULATION)
            and collision_free(poses, obstacle_tree))


def extend(parent, length, steer, obstacle_tree, bounds):
    """Roll out and validate one steering segment without snapping its end pose."""
    count = max(1, math.ceil(abs(length) / MOTION_RESOLUTION))
    distance = length / count
    pose = parent.poses[-1]
    poses = []
    for _ in range(count):
        pose = move(pose, distance, steer)
        poses.append(pose)
    if not valid_poses(poses, obstacle_tree, bounds):
        return None
    direction = 1 if length > 0.0 else -1
    trajectory = np.array(poses)
    articulation = np.abs(wrap_angle(trajectory[:, 2] - trajectory[:, 3]))
    cost = abs(length) * (1.0 if direction > 0 else BACK_COST)
    cost += STEER_COST * abs(steer) * abs(length)
    cost += STEER_CHANGE_COST * abs(steer - parent.steer)
    cost += ARTICULATION_COST * float(np.sum(articulation)) * abs(distance)
    if parent.direction and direction != parent.direction:
        cost += SWITCH_COST
    return Node(poses, [distance] * count, steer, direction, parent.cost + cost, parent)


def state_key(node):
    """Discretize all four coordinates; retain direction and steering for costs."""
    x, y, yaw, trailer_yaw = node.poses[-1]
    bins = round(2.0 * math.pi / YAW_RESOLUTION)
    return (round(x / XY_RESOLUTION), round(y / XY_RESOLUTION),
            round(wrap_angle(yaw) / YAW_RESOLUTION) % bins,
            round(wrap_angle(trailer_yaw) / YAW_RESOLUTION) % bins,
            node.direction, node.steer)


def at_goal(pose, goal):
    """Require the tractor pose and the independently integrated trailer heading."""
    return (math.hypot(pose[0] - goal[0], pose[1] - goal[1]) < 1e-6
            and abs(wrap_angle(pose[2] - goal[2])) < 1e-6
            and abs(wrap_angle(pose[3] - goal[3])) <= TRAILER_GOAL_TOLERANCE)


def analytic_expansion(current, goal, obstacle_tree, bounds):
    """Try Reeds-Shepp tractor connections, propagating and checking the trailer.

    A tractor connection alone is insufficient: reject it when the trailer
    misses its goal heading, collides, or exceeds the articulation limit.
    """
    pose = current.poses[-1]
    if math.hypot(pose[0] - goal[0], pose[1] - goal[1]) < 1e-8:
        # The existing Reeds-Shepp generator has singular coincident poses.
        # Ordinary motion primitives can still leave and return to this position.
        return None
    curvature = math.tan(MAX_STEER) / WHEEL_BASE
    try:
        candidates = rs.generate_path(pose[:3], goal[:3], curvature, 1e-6)
    except ZeroDivisionError:
        # Some exact circular configurations are singular in the shared
        # generator. Keep searching with ordinary motion primitives.
        return None
    best = None
    for candidate in candidates:
        node = current
        for length, mode in zip(candidate.lengths, candidate.ctypes):
            if abs(length) < 1e-10:
                continue
            steer = {"L": MAX_STEER, "R": -MAX_STEER, "S": 0.0}[mode]
            node = extend(node, length / curvature, steer, obstacle_tree, bounds)
            if node is None:
                break
        if node is not None and at_goal(node.poses[-1], goal):
            if best is None or node.cost < best.cost:
                best = node
    return best


def reconstruct_path(node, expanded_nodes):
    """Follow predecessor objects so reopening a grid cell cannot alter a path."""
    segments = []
    cost = node.cost
    while node is not None:
        segments.append(node)
        node = node.parent
    segments.reverse()
    return Path(np.array([pose for segment in segments for pose in segment.poses]),
                np.array([step for segment in segments for step in segment.distances]),
                np.array([segment.steer for segment in segments for _ in segment.poses]),
                cost, expanded_nodes)


def hybrid_a_star_planning(start, goal, obstacles, *, bounds=None, max_expansions=5000):
    """Plan for (x, y, tractor yaw, trailer yaw) poses and N x 2 obstacle points.

    ``bounds`` is (xmin, xmax, ymin, ymax) for the hitch [m]. By default it
    encloses the inputs plus SEARCH_MARGIN. Returns a Path, or None if an input
    pose is infeasible, the frontier is exhausted, or max_expansions is reached.
    No claim of completeness or optimality is made for this discretized search.
    Inputs are not modified. Returned tractor pose is exact to 1e-6; trailer yaw
    is within TRAILER_GOAL_TOLERANCE of the requested goal, not snapped to it.
    """
    start, goal = np.array(start, dtype=float), np.array(goal, dtype=float)
    obstacles = np.asarray(obstacles, dtype=float).reshape(-1, 2)
    start[2:], goal[2:] = wrap_angle(start[2:]), wrap_angle(goal[2:])
    obstacle_tree = cKDTree(obstacles)
    if bounds is None:
        points = np.vstack((start[:2], goal[:2], obstacles))
        low, high = points.min(axis=0) - SEARCH_MARGIN, points.max(axis=0) + SEARCH_MARGIN
        bounds = low[0], high[0], low[1], high[1]
    if not valid_poses([start, goal], obstacle_tree, bounds):
        return None
    current = Node([tuple(start)], [0.0])
    if at_goal(start, goal):
        return reconstruct_path(current, 0)
    serial = itertools.count()
    queue = [(0.0, next(serial), current)]
    best_cost = {state_key(current): 0.0}
    expanded = 0
    while queue and expanded < max_expansions:
        _, _, current = heapq.heappop(queue)
        if current.cost > best_cost[state_key(current)]:
            continue
        expanded += 1
        if at_goal(current.poses[-1], goal):
            return reconstruct_path(current, expanded)
        connection = analytic_expansion(current, goal, obstacle_tree, bounds)
        if connection is not None:
            return reconstruct_path(connection, expanded)
        for steer in np.linspace(-MAX_STEER, MAX_STEER, N_STEER):
            for direction in (1, -1):
                neighbor = extend(current, direction * PRIMITIVE_LENGTH,
                                  float(steer), obstacle_tree, bounds)
                if neighbor is None:
                    continue
                key = state_key(neighbor)
                if neighbor.cost >= best_cost.get(key, math.inf):
                    continue
                best_cost[key] = neighbor.cost
                pose = neighbor.poses[-1]
                heuristic = math.hypot(pose[0] - goal[0], pose[1] - goal[1])
                heapq.heappush(queue, (neighbor.cost + HEURISTIC_WEIGHT * heuristic,
                                      next(serial), neighbor))
    return None


def body_outline(pose, heading_column, front, rear):
    """Return a closed body polygon in world coordinates for plotting."""
    heading = pose[heading_column]
    local = np.array([[front, WIDTH / 2], [front, -WIDTH / 2],
                      [-rear, -WIDTH / 2], [-rear, WIDTH / 2], [front, WIDTH / 2]])
    rotation = np.array([[math.cos(heading), -math.sin(heading)],
                         [math.sin(heading), math.cos(heading)]])
    return local @ rotation.T + np.array(pose[:2])


def draw_frame(path, obstacles, start, goal, index):  # pragma: no cover
    """Draw the hitch path, trailer axle path, start/goal, and the two bodies."""
    plt.cla()
    plt.plot(obstacles[:, 0], obstacles[:, 1], ".k", label="Obstacles")
    plt.plot(path.poses[:, 0], path.poses[:, 1], "c--", label="Hitch path")
    trailer_axles = path.poses[:, :2] - TRAILER_LENGTH * np.column_stack(
        (np.cos(path.poses[:, 3]), np.sin(path.poses[:, 3])))
    plt.plot(trailer_axles[:index + 1, 0], trailer_axles[:index + 1, 1],
             color="tab:orange", label="Trailer axle path")
    for pose, label, color in ((start, "Start", "tab:green"), (goal, "Goal", "tab:red")):
        plt.plot(pose[0], pose[1], "o", color=color, label=label)
        plt.arrow(pose[0], pose[1], 2 * math.cos(pose[2]), 2 * math.sin(pose[2]),
                  color=color, width=0.1)
    for column, front, rear in ((2, TRACTOR_FRONT, TRACTOR_REAR),
                                 (3, TRAILER_FRONT, TRAILER_REAR)):
        outline = body_outline(goal, column, front, rear)
        plt.plot(outline[:, 0], outline[:, 1], "--", color="tab:red", alpha=0.4)
    pose = path.poses[index]
    for column, front, rear, color, label in (
        (2, TRACTOR_FRONT, TRACTOR_REAR, "tab:blue", "Tractor"),
        (3, TRAILER_FRONT, TRAILER_REAR, "tab:orange", "Trailer"),
    ):
        outline = body_outline(pose, column, front, rear)
        plt.plot(outline[:, 0], outline[:, 1], color=color, label=label)
    motion = "Reverse" if path.distances[index] < 0 else "Forward"
    angle = math.degrees(abs(wrap_angle(pose[2] - pose[3])))
    plt.title(f"Hybrid A* with trailer | {motion} | Articulation {angle:.1f}°")
    plt.xlabel("x [m]")
    plt.ylabel("y [m]")
    plt.gca().set_aspect("equal", adjustable="box")
    plt.xlim(obstacles[:, 0].min() - 2, obstacles[:, 0].max() + 2)
    plt.ylim(obstacles[:, 1].min() - 2, obstacles[:, 1].max() + 2)
    plt.grid(True)
    plt.legend(loc="upper left", fontsize=8)


def example():
    """Return a parking maneuver with point-sampled walls, in metres/radians."""
    start = [10.0, 10.0, 0.0, 0.0]
    goal = [30.0, 20.0, 0.0, 0.0]
    horizontal = np.arange(-2.0, 45.1, 0.5)
    vertical = np.arange(0.0, 35.1, 0.5)
    box_x, box_y = np.arange(17.0, 23.1, 0.5), np.arange(13.0, 17.1, 0.5)
    obstacles = np.vstack((
        np.column_stack((horizontal, np.zeros_like(horizontal))),
        np.column_stack((horizontal, np.full_like(horizontal, 35.0))),
        np.column_stack((np.full_like(vertical, -2.0), vertical)),
        np.column_stack((np.full_like(vertical, 45.0), vertical)),
        np.column_stack((box_x, np.full_like(box_x, 13.0))),
        np.column_stack((box_x, np.full_like(box_x, 17.0))),
        np.column_stack((np.full_like(box_y, 17.0), box_y)),
        np.column_stack((np.full_like(box_y, 23.0), box_y)),
    ))
    return start, goal, obstacles


def main():
    """Run the deterministic trailer example; return its path for inspection."""
    start, goal, obstacles = example()
    path = hybrid_a_star_planning(start, goal, obstacles)
    if path is None:
        raise RuntimeError("No trailer path found within the search limit")
    print(f"Path found after {path.expanded_nodes} expansions; cost {path.cost:.2f}")
    if show_animation:  # pragma: no cover
        plt.figure(figsize=(10, 6))
        for index in range(0, len(path.poses), 3):
            draw_frame(path, obstacles, start, goal, index)
            plt.pause(0.01)
        draw_frame(path, obstacles, start, goal, len(path.poses) - 1)
        plt.show()
    return path


if __name__ == "__main__":
    main()
