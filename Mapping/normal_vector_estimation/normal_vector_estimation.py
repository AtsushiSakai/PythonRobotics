import numpy as np
from matplotlib import pyplot as plt

from utils.plot import plot_3d_vector_arrow, plot_triangle, set_equal_3d_axis

show_animation = True


def calc_normal_vector(p1, p2, p3):
    """Calculate normal vector of triangle

    Parameters
    ----------
    p1 : np.array
        3D point
    p2 : np.array
        3D point
    p3 : np.array
        3D point

    Returns
    -------
    normal_vector : np.array
        normal vector (3,)

    """
    # calculate two vectors of triangle
    v1 = p2 - p1
    v2 = p3 - p1

    # calculate normal vector
    normal_vector = np.cross(v1, v2)

    # normalize vector
    normal_vector = normal_vector / np.linalg.norm(normal_vector)

    return normal_vector


def sample_3d_points_from_a_plane(num_samples, normal):
    points_2d = np.random.normal(size=(num_samples, 2))  # 2D points on a plane
    d = 0
    for i in range(len(points_2d)):
        point_3d = np.append(points_2d[i], 0)
        d += normal @ point_3d
    d /= len(points_2d)

    points_3d = np.zeros((len(points_2d), 3))
    for i in range(len(points_2d)):
        point_2d = np.append(points_2d[i], 0)
        projection_length = (d - normal @ point_2d) / np.linalg.norm(normal)
        points_3d[i] = point_2d + projection_length * normal

    return points_3d


def distance_to_plane(point, normal, origin):
    dot_product = np.dot(normal, point) - np.dot(normal, origin)
    if np.isclose(dot_product, 0):
        return 0.0
    else:
        distance = abs(dot_product) / np.linalg.norm(normal)
        return distance


def ransac_normal_vector_estimation(points_3d, inlier_radio_th=0.7,
                                    inlier_dist=0.1, p=0.99):
    """
    RANSAC based normal vector estimation

    Parameters
    ----------
    points_3d : np.array
        3D points (N, 3)
    inlier_radio_th : float
        Inlier ratio threshold. If inlier ratio is larger than this value,
        the iteration is stopped. Default is 0.7.
    inlier_dist : float
        Inlier distance threshold. If distance between points and estimated
        plane is smaller than this value, the point is inlier. Default is 0.1.
    p : float
         Probability that at least one of the sets of random samples does not
         include an outlier. If this probability is near 1, the iteration
         number is large. Default is 0.99.

    Returns
    -------
    center_vector : np.array
        Center of estimated plane. (3,)
    normal_vector : np.array
        Normal vector of estimated plane. (3,)

    """
    center = np.mean(points_3d, axis=0)

    max_iter = int(np.floor(np.log(1.0-p)/np.log(1.0-(1.0-inlier_radio_th)**3)))

    for ite in range(max_iter):
        # Random sampling
        sampled_ids = np.random.choice(points_3d.shape[0], size=3,
                                       replace=False)
        sampled_points = points_3d[sampled_ids, :]
        p1 = sampled_points[0, :]
        p2 = sampled_points[1, :]
        p3 = sampled_points[2, :]
        normal_vector = calc_normal_vector(p1, p2, p3)

        # calc inlier ratio
        n_inliner = 0
        for i in range(points_3d.shape[0]):
            p = points_3d[i, :]
            if distance_to_plane(p, normal_vector, center) <= inlier_dist:
                n_inliner += 1
        inlier_ratio = n_inliner / points_3d.shape[0]
        print(f"Iter:{ite}, {inlier_ratio=}")
        if inlier_ratio > inlier_radio_th:
            return center, normal_vector

    return center, None


def main1():
    p1 = np.array([0.0, 0.0, 1.0])
    p2 = np.array([1.0, 1.0, 0.0])
    p3 = np.array([0.0, 1.0, 0.0])

    center = np.mean([p1, p2, p3], axis=0)
    normal_vector = calc_normal_vector(p1, p2, p3)
    print(f"{center=}")
    print(f"{normal_vector=}")

    if show_animation:
        fig = plt.figure()
        ax = fig.add_subplot(projection='3d')
        set_equal_3d_axis(ax, [0.0, 2.5], [0.0, 2.5], [0.0, 3.0])
        plot_triangle(p1, p2, p3, ax)
        ax.plot(center[0], center[1], center[2], "ro")
        plot_3d_vector_arrow(ax, center, center + normal_vector)
        plt.show()


def main2(rng=None):
    true_normal = np.array([0, 1, 1])
    true_normal = true_normal / np.linalg.norm(true_normal)
    num_samples = 100
    noise_scale = 0.1

    points_3d = sample_3d_points_from_a_plane(num_samples, true_normal)
    # add random noise
    points_3d += np.random.normal(size=points_3d.shape, scale=noise_scale)

    print(f"{points_3d.shape=}")

    center, estimated_normal = ransac_normal_vector_estimation(
        points_3d, inlier_dist=noise_scale)

    if estimated_normal is None:
        print("Failed to estimate normal vector")
        return

    print(f"{true_normal=}")
    print(f"{estimated_normal=}")

    if show_animation:
        fig = plt.figure()
        ax = fig.add_subplot(projection='3d')
        ax.plot(points_3d[:, 0], points_3d[:, 1], points_3d[:, 2], ".r")
        plot_3d_vector_arrow(ax, center, center + true_normal)
        plot_3d_vector_arrow(ax, center, center + estimated_normal)
        set_equal_3d_axis(ax, [-3.0, 3.0], [-3.0, 3.0], [-3.0, 3.0])
        plt.title("RANSAC based Normal vector estimation")
        plt.show()


if __name__ == '__main__':
    # main1()
    main2()
