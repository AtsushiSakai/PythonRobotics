"""
Clothoidal Path Planner
Author: Daniel Ingram (daniel-s-ingram)
Reference paper: # Reference paper: https://www.researchgate.net/profile/Enrico-Bertolazzi/publication/237062806_Fast_and_accurate_G1_fitting_of_clothoid_curves/links/0046351b1a9d93cf56000000/Fast-and-accurate-G1-fitting-of-clothoid-curves.pdf
"""

import matplotlib.pyplot as plt
import numpy as np
import scipy.integrate as integrate
from collections import namedtuple
from scipy.optimize import fsolve
from math import cos, sin, pi, sqrt, atan2
from matplotlib import animation

Point = namedtuple("Point", ["x", "y"])

def draw_clothoids(theta1, num_clothoids=75, num_steps=100):
    p1 = Point(0, 0)
    p2 = Point(10, 0)
    theta2_vals = np.linspace(-pi, pi, num_clothoids)
    clothoids = [get_clothoid_points(p1, p2, theta1, theta2, num_steps) for theta2 in theta2_vals]
    
    fig = plt.figure(figsize=(10, 10))
    x_min, x_max, y_min, y_max = get_axes_limits(clothoids)
    axes = plt.axes(xlim=(x_min, x_max), ylim=(y_min, y_max))

    axes.plot(p1.x, p1.y, 'ro')
    axes.plot(p2.x, p2.y, 'ro')
    lines = [axes.plot([], [], 'b-')[0] for _ in range(num_clothoids)]

    def animate(i):
        for line, clothoid in zip(lines, clothoids):
            x = [p.x for p in clothoid[:i]]
            y = [p.y for p in clothoid[:i]]
            line.set_data(x, y)

        return lines

    anim = animation.FuncAnimation(fig, animate, frames=num_steps, interval=25, blit=True)
    anim.save('clothoid.gif', fps=30, writer="imagemagick")
    plt.show()

def get_clothoid_points(p1, p2, theta1, theta2, num_steps=100):
    dx = p2.x - p1.x
    dy = p2.y - p1.y
    r = sqrt(dx**2 + dy**2)

    phi = atan2(dy, dx)
    phi1 = normalize_angle(theta1 - phi)
    phi2 = normalize_angle(theta2 - phi)
    delta = phi2 - phi1

    try:
        A = solve_for_root(phi1, phi2, delta)
        L = compute_length(r, phi1, delta, A)
        curv = compute_curvature(delta, A, L)
        curv_rate = compute_curvature_rate(A, L)
    except Exception as e:
        print(f"Failed to generate clothoid points: {e}")
        return None

    points = []
    for s in np.linspace(0, L, num_steps):
        try:
            x = p1.x + s*X(curv_rate*s**2, curv*s, theta1)
            y = p1.y + s*Y(curv_rate*s**2, curv*s, theta1)
            points.append(Point(x, y))
        except Exception as e:
            print(f"Skipping failed clothoid point: {e}")

    return points

def X(a, b, c):
    return integrate.quad(lambda t: cos((a/2)*t**2 + b*t + c), 0, 1)[0]

def Y(a, b, c):
    return integrate.quad(lambda t: sin((a/2)*t**2 + b*t + c), 0, 1)[0]

def solve_for_root(theta1, theta2, delta):
    initial_guess = 3*(theta1 + theta2)
    return fsolve(lambda x: Y(2*x, delta - x, theta1), [initial_guess])

def compute_length(r, theta1, delta, A):
    return r / X(2*A, delta - A, theta1)

def compute_curvature(delta, A, L):
    return (delta - A) / L

def compute_curvature_rate(A, L):
    return 2 * A / (L**2)

def normalize_angle(angle_rad):
    return (angle_rad + pi) % (2 * pi) - pi

def get_axes_limits(clothoids):
    x_min = None
    x_max = None
    y_min = None
    y_max = None
    for clothoid in clothoids:
        for point in clothoid:
            if x_min is None or point.x < x_min:
                x_min = point.x

            if x_max is None or point.x > x_max:
                x_max = point.x

            if y_min is None or point.y < y_min:
                y_min = point.y

            if y_max is None or point.y > y_max:
                y_max = point.y

    x_offset = 0.1*(x_max - x_min)
    y_offset = 0.1*(y_max - y_min)

    x_min = x_min - x_offset
    x_max = x_max + x_offset
    y_min = y_min - y_offset
    y_max = y_max + y_offset

    return x_min, x_max, y_min, y_max

if __name__ == "__main__":
    draw_clothoids(0, 50)