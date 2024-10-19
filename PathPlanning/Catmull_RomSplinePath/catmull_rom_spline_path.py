"""
Path Planner with Catmull-Rom Spline
Author: Surabhi Gupta (@this_is_surabhi)
Source: http://graphics.cs.cmu.edu/nsp/course/15-462/Fall04/assts/catmullRom.pdf
"""

import sys
import pathlib
sys.path.append(str(pathlib.Path(__file__).parent.parent.parent))

import numpy as np
import matplotlib.pyplot as plt

def catmull_rom_point(t, p0, p1, p2, p3):
    """
    Parameters
    ----------
    t : float
        Parameter value (0 <= t <= 1) (0 <= t <= 1)
    p0, p1, p2, p3 : np.ndarray
        Control points for the spline segment

    Returns
    -------
    np.ndarray
        Calculated point on the spline
    """
    return 0.5 * ((2 * p1) +
                  (-p0 + p2) * t +
                  (2 * p0 - 5 * p1 + 4 * p2 - p3) * t**2 +
                  (-p0 + 3 * p1 - 3 * p2 + p3) * t**3)


def catmull_rom_spline(control_points, num_points):
    """
    Parameters
    ----------
    control_points : list
        List of control points
    num_points : int
        Number of points to generate on the spline

    Returns
    -------
    tuple
        x and y coordinates of the spline points
    """
    t_vals = np.linspace(0, 1, num_points)
    spline_points = []
    
    control_points = np.array(control_points)
    
    for i in range(len(control_points) - 1):
        if i == 0:
            p1, p2, p3 = control_points[i:i+3]
            p0 = p1
        elif i == len(control_points) - 2:
            p0, p1, p2 = control_points[i-1:i+2]
            p3 = p2
        else:
            p0, p1, p2, p3 = control_points[i-1:i+3]
        
        for t in t_vals:
            point = catmull_rom_point(t, p0, p1, p2, p3)
            spline_points.append(point)
    
    return np.array(spline_points).T


def main():
    print(__file__ + " start!!")

    way_points = [[-1.0, -2.0], [1.0, -1.0], [3.0, -2.0], [4.0, -1.0], [3.0, 1.0], [1.0, 2.0], [0.0, 2.0]]
    n_course_point = 100  
    spline_x, spline_y = catmull_rom_spline(way_points, n_course_point)

    plt.plot(spline_x,spline_y, '-r', label="Catmull-Rom Spline Path")
    plt.plot(np.array(way_points).T[0], np.array(way_points).T[1], '-og', label="Way points")
    plt.title("Catmull-Rom Spline Path")
    plt.grid(True)
    plt.legend()
    plt.axis("equal")
    plt.show()

if __name__ == '__main__':
    main()