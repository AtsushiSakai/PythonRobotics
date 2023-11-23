from collections.abc import Callable
from typing import NamedTuple

import torch
import torch.utils.data
from scipy import ndimage


def get_precomputed_sdf(map_tensor: torch.Tensor, cell_size: float) -> torch.Tensor:
    the_map = map_tensor
    inv_map = 1.0 - the_map

    if torch.max(inv_map) == 0.0:  # an empty map, i.e., no obsctales
        max_map_size = 2 * cell_size * max(the_map.shape)
        field = torch.ones(the_map.shape) * max_map_size
        return field

    map_dist = torch.tensor(ndimage.distance_transform_edt(the_map))
    inv_map_dist = torch.tensor(ndimage.distance_transform_edt(inv_map))

    field = map_dist - inv_map_dist
    field = field * cell_size
    return field


class Obstacle(NamedTuple):
    center: tuple[float, float]
    width: float
    height: float


def create_2dmap_data(
    origin: tuple[float, float],
    num_rows: int,
    num_cols: int,
    cell_size: float,
    obstacles: list[Obstacle],
) -> dict[str, torch.Tensor]:
    the_map = torch.ones(
        num_rows, num_cols
    )  # inverse occupancy seems like theseus' map_tensor format

    for obstacle in obstacles:
        center_ri = int((obstacle.center[1] - origin[1]) / cell_size)
        center_ci = int((obstacle.center[0] - origin[0]) / cell_size)
        half_width_rows = int((obstacle.width / cell_size) / 2)
        half_height_cols = int((obstacle.height / cell_size) / 2)
        the_map[
            center_ri - half_width_rows - 1 : center_ri + half_width_rows,
            center_ci - half_height_cols - 1 : center_ci + half_height_cols,
        ] = 0.0

    return {
        "map_tensor": the_map,
        "sdf_origin": torch.tensor(origin).double(),
        "cell_size": torch.tensor(cell_size).view(1),
        "sdf_data": get_precomputed_sdf(the_map, cell_size),
    }


class SquigglesDataset(torch.utils.data.Dataset):
    dataset_length = 1000

    def __init__(
        self,
        link_lengths: torch.Tensor,
        create_varying_vars: Callable[
            [int], tuple[torch.Tensor, torch.Tensor, list[Obstacle]]
        ],
        map_origin: tuple[float, float],
        map_num_rows: int,
        map_num_cols: int,
        map_cell_size: float,
    ) -> None:
        self.link_lengths_ = link_lengths
        self.create_varying_vars_ = create_varying_vars
        self.map_origin_ = map_origin
        self.map_num_rows_ = map_num_rows
        self.map_num_cols_ = map_num_cols
        self.map_cell_size_ = map_cell_size

    def __getitem__(self, idx: int) -> dict:
        init_joint_angles, target, obstacles = self.create_varying_vars_(idx)
        if idx % 2 == 0:
            init_joint_angles = torch.tensor([-1.0, -1.0, -1.0])
            target = torch.tensor([4.0, 0.0])
            obstacles = [Obstacle((1.5, -0.5), 1.0, 1.0)]
        else:
            init_joint_angles = torch.tensor([-2.0, 1.0, 1.0])
            target = torch.tensor([-5.0, 0.0])
            obstacles = [Obstacle((-1.5, -0.5), 1.0, 1.0)]

        data = create_2dmap_data(
            self.map_origin_,
            self.map_num_rows_,
            self.map_num_cols_,
            self.map_cell_size_,
            obstacles,
        )

        return {
            "id": idx,
            "link_lengths": self.link_lengths_,
            "init_joint_angles": init_joint_angles,
            "target": target,
            **data,
        }

    def __len__(self) -> int:
        return self.dataset_length
