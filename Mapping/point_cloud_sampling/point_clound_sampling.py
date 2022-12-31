
import matplotlib.pyplot as plt
import numpy as np
from collections import defaultdict


def voxel_point_sampling(original_points, voxel_size):
    voxel_dict = defaultdict(list)
    for i in range(original_points.shape[0]):
        xyz = original_points[i, :]
        xyz_index = tuple(xyz // voxel_size)
        voxel_dict[xyz_index].append(xyz)
    points = np.vstack([np.mean(v, axis=0) for v in voxel_dict.values()])
    return points


def main():
    np.random.seed(1234)
    fig = plt.figure()
    ax = fig.add_subplot(projection='3d')

    n_points = 1000
    rng = np.random.default_rng()

    x = rng.normal(0.0, 10.0, n_points)
    y = rng.normal(0.0, 1.0, n_points)
    z = rng.normal(0.0, 10.0, n_points)
    ax.scatter(x, y, z, marker=".", label="Original points")
    original_points = np.vstack((x, y, z)).T
    print(f"{original_points.shape=}")

    print("Voxel point sampling")
    voxel_size = 20.0
    filtered_points = voxel_point_sampling(original_points, voxel_size)
    print(f"{filtered_points.shape=}")
    ax.scatter(filtered_points[:, 0], filtered_points[:, 1],
               filtered_points[:, 2], marker="o", label="Filtered points")
    plt.legend()
    plt.title("Voxel point sampling")
    plt.axis("equal")
    plt.show()


if __name__ == '__main__':
    main()
