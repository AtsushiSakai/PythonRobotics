import conftest
import numpy as np
from PathPlanning.ElasticBands.elastic_bands import ElasticBands


def test_1():
    path = np.load("PathPlanning/ElasticBands/path.npy")
    obstacles_points = np.load("PathPlanning/ElasticBands/obstacles.npy")
    obstacles = np.zeros((500, 500))
    for x, y in obstacles_points:
        size = 30  # Side length of the square
        half_size = size // 2
        x_start = max(0, x - half_size)
        x_end = min(obstacles.shape[0], x + half_size)
        y_start = max(0, y - half_size)
        y_end = min(obstacles.shape[1], y + half_size)
        obstacles[x_start:x_end, y_start:y_end] = 1
    elastic_bands = ElasticBands(path, obstacles)
    elastic_bands.update_bubbles()


if __name__ == "__main__":
    conftest.run_this_test(__file__)
