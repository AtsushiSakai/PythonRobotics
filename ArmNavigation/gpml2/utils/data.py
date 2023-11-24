from collections.abc import Callable
from math import floor
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

    map_dist = torch.tensor(ndimage.distance_transform_edt(inv_map))
    inv_map_dist = torch.tensor(ndimage.distance_transform_edt(the_map))

    field = map_dist - inv_map_dist
    field = field * cell_size
    return field


class Obstacle(NamedTuple):
    center: tuple[float, float]
    width: float
    height: float


def create_2dmap_data(
    origin: tuple[float, float],
    nrows: int,
    ncols: int,
    cell_size: float,
    obstacles: list[Obstacle],
) -> dict[str, torch.Tensor]:
    the_map = torch.zeros(
        nrows, ncols
    )  # inverse occupancy seems like theseus' map_tensor format

    origin_x, origin_y = origin
    obstacles_tensor = torch.zeros((len(obstacles), 5, 2))
    for i, obstacle in enumerate(obstacles):
        center_x, center_y = obstacle.center
        half_width = obstacle.width / 2
        half_height = obstacle.height / 2

        center_ri = floor((center_x - origin_x) / cell_size)
        center_ci = floor((center_y - origin_y) / cell_size)
        half_rows = floor(half_width / cell_size)
        half_cols = floor(half_height / cell_size)
        the_map[
            center_ri - half_rows : center_ri + half_rows + 1,
            center_ci - half_cols : center_ci + half_cols + 1,
        ] = 1.0

        top_left = ((center_x - half_width), (center_y - half_height))
        top_right = ((center_x + half_width), (center_y - half_height))
        bottom_right = ((center_x + half_width), (center_y + half_height))
        bottom_left = ((center_x - half_width), (center_y + half_height))
        obstacles_tensor[i] = torch.tensor(
            [top_left, top_right, bottom_right, bottom_left, top_left]
        )

    return {
        "map_tensor": the_map,
        "sdf_origin": torch.tensor(origin).double(),
        "cell_size": torch.tensor(cell_size).view(1),
        "sdf_data": get_precomputed_sdf(the_map, cell_size).T,
        "obstacles_tensor": obstacles_tensor,
    }


class LinkArmDataset(torch.utils.data.Dataset):
    dataset_length = 1000

    def __init__(
        self,
        link_lengths: torch.Tensor,
        create_varying_vars: Callable[
            [int], tuple[torch.Tensor, torch.Tensor, list[Obstacle], torch.Tensor]
        ],
        map_origin: tuple[float, float],
        map_nrows: int,
        map_ncols: int,
        map_cell_size: float,
    ) -> None:
        self.link_lengths_ = link_lengths
        self.create_varying_vars_ = create_varying_vars
        self.map_origin_ = map_origin
        self.map_nrows_ = map_nrows
        self.map_ncols_ = map_ncols
        self.map_cell_size_ = map_cell_size

    def __getitem__(self, idx: int) -> dict:
        (
            init_joint_angles,
            target,
            obstacles,
            expert_trajectory,
        ) = self.create_varying_vars_(idx)

        data = create_2dmap_data(
            self.map_origin_,
            self.map_nrows_,
            self.map_ncols_,
            self.map_cell_size_,
            obstacles,
        )

        return {
            **data,
            "id": idx,
            "link_lengths": self.link_lengths_,
            "init_joint_angles": init_joint_angles,
            "target": target,
            "expert_trajectory": expert_trajectory,
        }

    def __len__(self) -> int:
        return self.dataset_length
