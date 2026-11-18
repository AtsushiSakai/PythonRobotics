"""
Probabilistic Line Extraction from 2D Point Cloud Data

Two algorithms implemented:
  1. RANSAC-based line extraction
  2. Split-and-Merge line extraction

Typical use case: extracting wall/obstacle lines from 2D LiDAR scans
for robot mapping and navigation.

Reference:
  Probabilistic Robotics - Sebastian Thrun, Wolfram Burgard, Dieter Fox
  Chapter 6: Robot Perception

Author: Akshay Shailesh Raikar
"""

import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
from dataclasses import dataclass
from typing import Optional


# ─── DATA STRUCTURES ──────────────────────────────────────────────────────────

@dataclass
class Line:
    """Represents a line segment extracted from point cloud."""
    x_start: float
    y_start: float
    x_end: float
    y_end: float
    a: float          # line equation: ax + by + c = 0
    b: float
    c: float
    inlier_count: int = 0

    def length(self) -> float:
        return np.hypot(self.x_end - self.x_start, self.y_end - self.y_start)

    def distance_to_point(self, x: float, y: float) -> float:
        """Perpendicular distance from point to infinite line."""
        return abs(self.a * x + self.b * y + self.c) / np.hypot(self.a, self.b)


# ─── LINE FITTING ─────────────────────────────────────────────────────────────

def fit_line(points: np.ndarray):
    """
    Fit a line to a set of 2D points using total least squares.
    Returns (a, b, c) for line equation ax + by + c = 0.
    """
    cx, cy = np.mean(points[:, 0]), np.mean(points[:, 1])
    dx = points[:, 0] - cx
    dy = points[:, 1] - cy
    sxx, sxy, syy = np.sum(dx**2), np.sum(dx * dy), np.sum(dy**2)

    # eigenvector of covariance matrix
    angle = 0.5 * np.arctan2(2 * sxy, sxx - syy)
    a = -np.sin(angle)
    b = np.cos(angle)
    c = -(a * cx + b * cy)
    return a, b, c


def make_line_segment(points: np.ndarray, a: float, b: float, c: float) -> Line:
    """Project inlier points onto the line and return the segment endpoints."""
    # direction vector along the line
    dx, dy = b, -a
    # project each point onto the line direction
    t = points[:, 0] * dx + points[:, 1] * dy
    t_min, t_max = np.min(t), np.max(t)
    # reconstruct endpoints
    ref_x = -a * c / (a**2 + b**2)
    ref_y = -b * c / (a**2 + b**2)
    x_start = ref_x + t_min * dx
    y_start = ref_y + t_min * dy
    x_end   = ref_x + t_max * dx
    y_end   = ref_y + t_max * dy
    return Line(x_start, y_start, x_end, y_end, a, b, c, len(points))


# ─── RANSAC LINE EXTRACTION ───────────────────────────────────────────────────

def ransac_line_extraction(
    points: np.ndarray,
    distance_threshold: float = 0.1,
    min_inliers: int = 10,
    max_iterations: int = 100,
    max_lines: int = 10,
    min_remaining: int = 10,
) -> list[Line]:
    """
    Extract multiple lines from a 2D point cloud using iterative RANSAC.

    Each iteration finds the dominant line, removes its inliers,
    then repeats on the remaining points.

    Args:
        points:             (N, 2) array of [x, y] points
        distance_threshold: max perpendicular distance to count as inlier
        min_inliers:        minimum inliers for a valid line
        max_iterations:     RANSAC iterations per line
        max_lines:          maximum lines to extract
        min_remaining:      stop when fewer points remain

    Returns:
        List of Line objects sorted by inlier count descending.
    """
    remaining = points.copy()
    lines = []

    while len(remaining) >= min_remaining and len(lines) < max_lines:
        best_inliers = None
        best_line_params = None
        best_count = 0

        n = len(remaining)
        for _ in range(max_iterations):
            # sample 2 random points
            idx = np.random.choice(n, 2, replace=False)
            sample = remaining[idx]

            if np.allclose(sample[0], sample[1]):
                continue

            a, b, c = fit_line(sample)
            norm = np.hypot(a, b)
            if norm < 1e-10:
                continue

            # find inliers
            dists = np.abs(a * remaining[:, 0] + b * remaining[:, 1] + c) / norm
            inlier_mask = dists < distance_threshold
            count = np.sum(inlier_mask)

            if count > best_count:
                best_count = count
                best_inliers = inlier_mask
                best_line_params = (a, b, c)

        if best_count < min_inliers:
            break

        # refit on all inliers
        inlier_pts = remaining[best_inliers]
        a, b, c = fit_line(inlier_pts)
        line = make_line_segment(inlier_pts, a, b, c)
        lines.append(line)

        # remove inliers from remaining
        remaining = remaining[~best_inliers]

    return sorted(lines, key=lambda l: l.inlier_count, reverse=True)


# ─── SPLIT AND MERGE LINE EXTRACTION ─────────────────────────────────────────

def split_and_merge(
    points: np.ndarray,
    distance_threshold: float = 0.1,
    min_points: int = 5,
) -> list[Line]:
    """
    Extract lines using the Split-and-Merge algorithm.

    Split: recursively split point sequences at the point with
           maximum perpendicular distance to the current line.
    Merge: merge collinear adjacent segments.

    Args:
        points:             (N, 2) ordered array (e.g. from a LiDAR scan)
        distance_threshold: max distance before splitting
        min_points:         minimum points to form a line

    Returns:
        List of Line objects.
    """
    lines = []
    _split(points, distance_threshold, min_points, lines)
    lines = _merge(lines, distance_threshold)
    return lines


def _split(points, threshold, min_pts, result):
    if len(points) < min_pts:
        return

    a, b, c = fit_line(points)
    norm = np.hypot(a, b)

    dists = np.abs(a * points[:, 0] + b * points[:, 1] + c) / norm
    max_idx = np.argmax(dists)
    max_dist = dists[max_idx]

    if max_dist > threshold and max_idx > 0 and max_idx < len(points) - 1:
        # split at max distance point
        _split(points[:max_idx + 1], threshold, min_pts, result)
        _split(points[max_idx:],     threshold, min_pts, result)
    else:
        a, b, c = fit_line(points)
        line = make_line_segment(points, a, b, c)
        result.append(line)


def _merge(lines: list[Line], threshold: float) -> list[Line]:
    """Merge adjacent lines that are approximately collinear."""
    if len(lines) < 2:
        return lines

    merged = True
    while merged:
        merged = False
        new_lines = [lines[0]]
        for i in range(1, len(lines)):
            prev = new_lines[-1]
            curr = lines[i]
            # check if endpoint of prev is close to line of curr
            d1 = curr.distance_to_point(prev.x_end, prev.y_end)
            d2 = prev.distance_to_point(curr.x_start, curr.y_start)
            if d1 < threshold and d2 < threshold:
                # merge: combine all points implicitly via endpoints
                combined_pts = np.array([
                    [prev.x_start, prev.y_start],
                    [prev.x_end,   prev.y_end],
                    [curr.x_start, curr.y_start],
                    [curr.x_end,   curr.y_end],
                ])
                a, b, c = fit_line(combined_pts)
                m = make_line_segment(combined_pts, a, b, c)
                m.inlier_count = prev.inlier_count + curr.inlier_count
                new_lines[-1] = m
                merged = True
            else:
                new_lines.append(curr)
        lines = new_lines

    return lines


# ─── POINT CLOUD GENERATOR ────────────────────────────────────────────────────

def generate_test_point_cloud(noise_std: float = 0.03, seed: int = 42) -> np.ndarray:
    """
    Generate a synthetic 2D point cloud simulating a LiDAR scan
    in a rectangular room with random noise.
    """
    rng = np.random.default_rng(seed)
    points = []

    # wall segments: (x1, y1, x2, y2, num_pts)
    walls = [
        (0.0, 0.0, 4.0, 0.0, 40),   # bottom
        (4.0, 0.0, 4.0, 3.0, 30),   # right
        (4.0, 3.0, 0.0, 3.0, 40),   # top
        (0.0, 3.0, 0.0, 0.0, 30),   # left
        (1.0, 1.0, 3.0, 1.0, 20),   # inner obstacle
        (1.5, 1.5, 1.5, 2.5, 15),   # inner obstacle
    ]

    for x1, y1, x2, y2, n in walls:
        t = np.linspace(0, 1, n)
        xs = x1 + t * (x2 - x1) + rng.normal(0, noise_std, n)
        ys = y1 + t * (y2 - y1) + rng.normal(0, noise_std, n)
        points.extend(zip(xs, ys))

    # random outliers
    n_out = 20
    points.extend(zip(
        rng.uniform(0, 4, n_out),
        rng.uniform(0, 3, n_out)
    ))

    return np.array(points)


# ─── VISUALIZATION ────────────────────────────────────────────────────────────

def plot_results(points: np.ndarray,
                 ransac_lines: list[Line],
                 sam_lines: list[Line]) -> None:

    fig, axes = plt.subplots(1, 3, figsize=(15, 5))
    fig.suptitle("Probabilistic Line Extraction from 2D Point Cloud",
                 fontsize=13, fontweight='bold')

    colors = plt.cm.tab10.colors

    # ── raw point cloud
    ax = axes[0]
    ax.scatter(points[:, 0], points[:, 1], s=8, color='#333333', alpha=0.6)
    ax.set_title("Input Point Cloud", fontsize=11)
    ax.set_aspect('equal')
    ax.set_xlabel("x (m)")
    ax.set_ylabel("y (m)")
    ax.grid(True, linestyle='--', alpha=0.4)

    # ── RANSAC result
    ax = axes[1]
    ax.scatter(points[:, 0], points[:, 1], s=8, color='#cccccc', alpha=0.5)
    for i, line in enumerate(ransac_lines):
        c = colors[i % len(colors)]
        ax.plot([line.x_start, line.x_end],
                [line.y_start, line.y_end],
                color=c, linewidth=2.5,
                label=f"Line {i+1} ({line.inlier_count} pts)")
        ax.scatter([line.x_start, line.x_end],
                   [line.y_start, line.y_end],
                   color=c, s=30, zorder=5)
    ax.set_title(f"RANSAC  —  {len(ransac_lines)} lines found", fontsize=11)
    ax.set_aspect('equal')
    ax.set_xlabel("x (m)")
    ax.set_ylabel("y (m)")
    ax.legend(fontsize=7, loc='upper right')
    ax.grid(True, linestyle='--', alpha=0.4)

    # ── Split-and-Merge result
    ax = axes[2]
    ax.scatter(points[:, 0], points[:, 1], s=8, color='#cccccc', alpha=0.5)
    for i, line in enumerate(sam_lines):
        c = colors[i % len(colors)]
        ax.plot([line.x_start, line.x_end],
                [line.y_start, line.y_end],
                color=c, linewidth=2.5,
                label=f"Line {i+1}")
        ax.scatter([line.x_start, line.x_end],
                   [line.y_start, line.y_end],
                   color=c, s=30, zorder=5)
    ax.set_title(f"Split-and-Merge  —  {len(sam_lines)} lines found", fontsize=11)
    ax.set_aspect('equal')
    ax.set_xlabel("x (m)")
    ax.set_ylabel("y (m)")
    ax.legend(fontsize=7, loc='upper right')
    ax.grid(True, linestyle='--', alpha=0.4)

    plt.tight_layout()
    plt.savefig("probabilistic_line_extraction.png", dpi=150, bbox_inches='tight')
    plt.show()


# ─── MAIN ─────────────────────────────────────────────────────────────────────

def main():
    print("Probabilistic Line Extraction Demo")
    print("=" * 40)

    # generate synthetic LiDAR point cloud
    points = generate_test_point_cloud(noise_std=0.03)
    print(f"Point cloud: {len(points)} points")

    # RANSAC
    print("\n[RANSAC] Running...")
    ransac_lines = ransac_line_extraction(
        points,
        distance_threshold=0.05,
        min_inliers=10,
        max_iterations=200,
        max_lines=10,
    )
    print(f"[RANSAC] Found {len(ransac_lines)} lines")
    for i, line in enumerate(ransac_lines):
        print(f"  Line {i+1}: length={line.length():.2f}m, inliers={line.inlier_count}")

    # Split-and-Merge (needs ordered points — sort by angle from centroid)
    print("\n[Split-and-Merge] Running...")
    cx, cy = np.mean(points[:, 0]), np.mean(points[:, 1])
    angles = np.arctan2(points[:, 1] - cy, points[:, 0] - cx)
    ordered = points[np.argsort(angles)]
    sam_lines = split_and_merge(
        ordered,
        distance_threshold=0.08,
        min_points=5,
    )
    print(f"[Split-and-Merge] Found {len(sam_lines)} lines")
    for i, line in enumerate(sam_lines):
        print(f"  Line {i+1}: length={line.length():.2f}m")

    # plot
    plot_results(points, ransac_lines, sam_lines)
    print("\nPlot saved: probabilistic_line_extraction.png")


if __name__ == "__main__":
    main()
