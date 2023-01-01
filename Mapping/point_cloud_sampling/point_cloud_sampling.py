"""
Point cloud sampling example codes. This code supports
- voxel point sampling
-

"""
import matplotlib.pyplot as plt
import numpy as np
from collections import defaultdict

do_plot = True


def voxel_point_sampling(original_points, voxel_size):
    voxel_dict = defaultdict(list)
    for i in range(original_points.shape[0]):
        xyz = original_points[i, :]
        xyz_index = tuple(xyz // voxel_size)
        voxel_dict[xyz_index].append(xyz)
    points = np.vstack([np.mean(v, axis=0) for v in voxel_dict.values()])
    return points


def main():
    n_points = 1000
    rng = np.random.default_rng(1234)

    x = rng.normal(0.0, 10.0, n_points)
    y = rng.normal(0.0, 1.0, n_points)
    z = rng.normal(0.0, 10.0, n_points)
    original_points = np.vstack((x, y, z)).T
    print(f"{original_points.shape=}")
    print("Voxel point sampling")
    voxel_size = 20.0
    voxel_sampling_points = voxel_point_sampling(original_points, voxel_size)
    print(f"{voxel_sampling_points.shape=}")

    if do_plot:
        fig = plt.figure()
        ax = fig.add_subplot(projection='3d')
        ax.scatter(x, y, z, marker=".", label="Original points")

        ax.scatter(voxel_sampling_points[:, 0], voxel_sampling_points[:, 1],
                   voxel_sampling_points[:, 2], marker="o", label="Filtered points")
        plt.legend()
        plt.title("Voxel point sampling")
        plt.axis("equal")
        plt.show()


if __name__ == '__main__':
    main()
