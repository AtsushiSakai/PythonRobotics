import sys
import warnings

import torch
import torch.utils.data
from planning.data import LinkArmDataset, Obstacle
from planning.kinematics import inverse_kinematics, resample_trajectory
from planning.viz import generate_figs


def create_dataset() -> LinkArmDataset:
    link_lengths = torch.tensor([3.0, 2.0, 1.0])

    def create_varying_vars(
        idx: int
    ) -> tuple[torch.Tensor, torch.Tensor, list[Obstacle], torch.Tensor]:
        trajectory_len = 10
        if idx % 2 == 0:
            init_joint_angles = torch.tensor([-1.0, -1.0, -1.0])
            target = torch.tensor([4.0, 0.0])
            obstacles = [Obstacle((2.25, -0.5), 1.0, 1.0)]
            midpoint_joint_angles = torch.tensor([-1.0, 0.5, 0.5])
        else:
            init_joint_angles = torch.tensor([-2.0, 1.0, 1.0])
            target = torch.tensor([-5.0, 0.0])
            obstacles = [Obstacle((-2.75, -0.5), 1.0, 1.0)]
            midpoint_joint_angles = torch.tensor([-2.0, -0.5, -0.5])

        target_joint_angles, success = inverse_kinematics(
            link_lengths, midpoint_joint_angles, target
        )
        if not success:
            warnings.warn("IK failed!")
        trajectory = torch.stack(
            [init_joint_angles, midpoint_joint_angles, target_joint_angles]
        )
        expert_trajectory = torch.tensor(
            resample_trajectory(trajectory, trajectory_len)
        )

        return init_joint_angles, target, obstacles, expert_trajectory

    dataset = LinkArmDataset(
        link_lengths=torch.tensor([3.0, 2.0, 1.0]),
        create_varying_vars=create_varying_vars,
        map_origin=(-9.0, -9.0),
        map_num_rows=180,
        map_num_cols=180,
        map_cell_size=0.1,
    )
    return dataset


if __name__ == "__main__":
    debug = "-d" in sys.argv[1:]

    dataset = create_dataset()
    batch_size = 2
    data_loader = torch.utils.data.DataLoader(dataset, batch_size)

    batch = next(iter(data_loader))
    if debug:
        for k, v in batch.items():
            if k != "id":
                print(f"{k:20s}: {v.shape}", type(v))

        figs = generate_figs(
            link_lengths=batch["link_lengths"],
            init_joint_angles=batch["init_joint_angles"],
            target=batch["target"],
            obstacles_tensor=batch["obstacles_tensor"],
            expert_trajectory=batch["expert_trajectory"],
        )
        figs[0].savefig("trajectory0.png")
        figs[1].savefig("trajectory1.png")

    print("Done!")
