"""
Distance/Path Transform Wavefront Coverage Path Planner

author: Todd Tang
paper: Planning paths of complete coverage of an unstructured environment by a mobile robot - Zelinsky et.al. 
link: http://pinkwink.kr/attachment/cfile3.uf@1354654A4E8945BD13FE77.pdf
"""

import os

import cv2

import matplotlib.pyplot as plt
import numpy as np

from scipy import ndimage


def transform(gridmap:np.ndarray, src:tuple, distance_type='chessboard', transform_type='path', alpha=0.01):
    """transform
    
    calculating transform of transform_type from source in the given distance_type 

    :param gridmap: 2d binary map
    :param src: distance transform source
    :param distance_type: type of distance used
    :param transform_type: type of transform used
    :param alpha: weight for Obstacle Transform used in path_transform calculating 
    """

    nrows, ncols = gridmap.shape

    if nrows == 0 or ncols == 0:
        print('Empty gridmap')
        return

    order = [[0, 1], [1, 1], [1, 0], [1, -1], [0, -1], [-1, -1], [-1, 0], [-1, 1]]
    
    if distance_type == 'chessboard':
        cost = [1, 1, 1, 1, 1, 1, 1, 1]
    elif distance_type == 'eculidean':
        cost = [1, np.sqrt(2), 1, np.sqrt(2), 1, np.sqrt(2), 1, np.sqrt(2)]
    else:
        print('Unsupported distance type.')
        return

    T = float('inf') * np.ones_like(gridmap, dtype=np.float)
    T[src[0], src[1]] = 0
    
    if transform_type == 'distance':
        eT = np.zeros_like(gridmap)
    elif transform_type == 'path':
        eT = ndimage.distance_transform_cdt(1-gridmap, distance_type)
    else:
        print('Unsupported transform type.')
        return

    # set obstacle T value to infinity
    for i in range(nrows):
        for j in range(ncols):
            if gridmap[i][j] == 1.0:
                T[i][j] = float('inf')
    
    is_visited = np.zeros_like(T, dtype=bool)
    is_visited[src[0], src[1]] = True
    
    queue = [src]
    calculated = [(src[0]-1)*ncols + src[1]]

    while queue != []:
        i, j = queue.pop(0)
        for k, inc in enumerate(order):
            ni = i + inc[0]
            nj = j + inc[1]
            if ni >= 0 and ni < nrows and nj >= 0 and nj < ncols and not gridmap[ni][nj]:
                is_visited[i][j] = True
                T[i][j] = min(T[i][j], T[ni][nj] + cost[k] + alpha * eT[ni][nj])
                if not is_visited[ni][nj] and ((ni-1)*ncols + nj) not in calculated:
                    queue.append((ni, nj))
                    calculated.append((ni-1)*ncols + nj)

    return T


def wavefront(T:np.ndarray, start:tuple, goal:tuple):
    """wavefront

    performing wavefront coverage path planning

    :param T: the transform matrix
    :param start: start point of planning
    :param goal: goal point of planning
    """

    path = []

    nrows, ncols = T.shape

    if start[0] >= goal[0] and start[1] >= goal[1]:
        order = [[1, 0], [0, 1], [-1, 0], [0, -1], [1, 1], [1, -1], [-1, 1], [-1, -1]]
    elif start[0] <= goal[0] and start[1] >= goal[1]:
        order = [[-1, 0], [0, 1], [1, 0], [0, -1], [-1, 1], [-1, -1], [1, 1], [1, -1]]
    elif start[0] >= goal[0] and start[1] <= goal[1]:
        order = [[1, 0], [0, -1], [-1, 0], [0, 1], [1, -1], [-1, -1], [1, 1], [-1, 1]]
    elif start[0] <= goal[0] and start[1] <= goal[1]:
        order = [[-1, 0], [0, -1], [0, 1], [1, 0], [-1, -1], [-1, 1], [1, -1], [1, 1]]
    else:
        return
    
    cur = start
    is_visited = np.zeros_like(T, dtype=bool)

    while cur != goal:
        i, j = cur
        path.append((i, j))
        is_visited[i][j] = True

        max_T = float('-inf')
        imax = (-1, -1)
        ilast = 0
        for ilast in range(len(path)):
            cur = path[-1-ilast]
            for ci, cj in order:
                ni, nj = cur[0] + ci, cur[1] + cj
                if ni >= 0 and ni < nrows and nj >= 0 and nj < ncols \
                    and not is_visited[ni][nj] and T[ni][nj] != float('inf'):
                    if T[ni][nj] > max_T:
                        imax = (ni, nj)
                        max_T = T[ni][nj]

            if imax != (-1, -1):
                break

        if imax == (-1, -1):
            break
        else:
            cur = imax
            if ilast != 0:
                print('backtracing to [%d, %d]'%(cur[0], cur[1]))
    
    path.append((goal))

    return path


def viz_plan(grid_map, start, goal, path, resolution):
    oy, ox = start
    gy, gx = goal
    px, py = np.transpose(np.flipud(np.fliplr((path))))

    if not do_animation:
        plt.imshow(grid_map, cmap='Greys')
        plt.plot(ox, oy, "-xy")
        plt.plot(px, py, "-r")
        plt.plot(gx, gy, "-pg")
        plt.show()
    else:
        for ipx, ipy in zip(px, py):
            plt.cla()
            # for stopping simulation with the esc key.
            plt.gcf().canvas.mpl_connect(
                'key_release_event',
                lambda event: [exit(0) if event.key == 'escape' else None])
            plt.imshow(grid_map, cmap='Greys')
            plt.plot(ox, oy, "-xb")
            plt.plot(px, py, "-r")
            plt.plot(gx, gy, "-pg")
            plt.plot(ipx, ipy, "or")
            plt.axis("equal")
            plt.grid(True)
            plt.pause(0.1)


if __name__ == "__main__":
    dir_path = os.path.dirname(os.path.realpath(__file__))
    img = cv2.imread(os.path.join(dir_path, 'map', 'test.png'), cv2.IMREAD_GRAYSCALE)
    img = 1 - img / 255

    start = (43, 0)
    goal = (0, 0)

    do_animation = True
    
    # distance transform wavefront
    DT = transform(img, goal, transform_type='distance')
    DT_path = wavefront(DT, start, goal)
    viz_plan(img, start, goal, DT_path, 1)

    # path transform wavefront
    PT = transform(img, goal, transform_type='path', alpha=0.01)
    PT_path = wavefront(PT, start, goal)
    viz_plan(img, start, goal, PT_path, 1)