import conftest
import numpy as np
from PathPlanning.ElasticBands.elastic_bands import ElasticBands


def test_1():
    path = np.load("PathPlanning/ElasticBands/points.npy")
    obstacles = np.load("PathPlanning/ElasticBands/obstacles.npy")
    elastic_bands = ElasticBands(path, obstacles)
    elastic_bands.update_bubbles()


if __name__ == "__main__":
    conftest.run_this_test(__file__)
