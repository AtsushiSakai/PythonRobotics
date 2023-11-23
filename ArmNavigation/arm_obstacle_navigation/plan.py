import sys

import torch
import torch.utils.data
from planning.data import Obstacle, SquigglesDataset
from planning.viz import generate_figs


def create_dataset(batch_size: int = 2) -> dict[str, torch.Tensor]:
    def create_varying_vars(
        idx: int
    ) -> tuple[torch.Tensor, torch.Tensor, list[Obstacle]]:
        if idx % 2 == 0:
            init_joint_angles = torch.tensor([-1.0, -1.0, -1.0])
            target = torch.tensor([4.0, 0.0])
            obstacles = [Obstacle((1.5, -0.5), 1.0, 1.0)]
        else:
            init_joint_angles = torch.tensor([-2.0, 1.0, 1.0])
            target = torch.tensor([-5.0, 0.0])
            obstacles = [Obstacle((-1.5, -0.5), 1.0, 1.0)]
        return init_joint_angles, target, obstacles

    dataset = SquigglesDataset(
        link_lengths=torch.tensor([3.0, 2.0, 1.0]),
        create_varying_vars=create_varying_vars,
        map_origin=(-9.0, -9.0),
        map_num_rows=180,
        map_num_cols=180,
        map_cell_size=0.1,
    )
    data_loader = torch.utils.data.DataLoader(dataset, batch_size)

    batch = next(iter(data_loader))
    return batch


if __name__ == "__main__":
    batch_size = 2
    debug = "-d" in sys.argv[1:]

    batch = create_dataset(batch_size)
    if debug:
        for k, v in batch.items():
            if k != "id":
                print(f"{k:20s}: {v.shape}", type(v))

        figs = generate_figs(
            link_lengths=batch["link_lengths"],
            init_joint_angles=batch["init_joint_angles"],
            target=batch["target"],
            obstacles_tensor=batch["obstacles_tensor"],
        )
        figs[0].savefig("trajectory0.png")

    print("Done")