import matplotlib.pyplot as plt
import torch
from matplotlib.axes import Axes
from matplotlib.figure import Figure
from planning.kinematics import forward_kinematics


def draw_robot(
    ax: Axes,
    joints_xy: torch.Tensor,
) -> None:
    ax.plot(joints_xy[:, 0], joints_xy[:, 1], "r-")
    ax.plot(
        joints_xy[:, 0],
        joints_xy[:, 1],
        "ko",
    )


def generate_figs(
    link_lengths: torch.Tensor,
    init_joint_angles: torch.Tensor,
    target: torch.Tensor,
    obstacles_tensor: torch.Tensor,
    trajectory: torch.Tensor,
) -> list[Figure]:
    figs: list[Figure] = []
    batch_size = link_lengths.shape[0]
    for i in range(batch_size):
        fig, axs = plt.subplots(1, 1)

        joints_xy = forward_kinematics(link_lengths[i], init_joint_angles[i])
        draw_robot(axs, joints_xy)

        for j in range(trajectory.shape[2]):
            cur_trajectory = trajectory[i, :, j]
            joints_xy = forward_kinematics(link_lengths[i], cur_trajectory)
            draw_robot(axs, joints_xy)

        axs.plot(target[i, 0], target[i, 1], "go")

        for obstacle in obstacles_tensor[i]:
            axs.plot(obstacle[:, 0], obstacle[:, 1], "k-")

        lim_val = torch.sum(link_lengths[i]) * 1.5
        axs.set(xlim=(-lim_val, lim_val), ylim=(-lim_val, lim_val))
        axs.grid(visible=True)
        axs.set_xlabel("x")
        axs.set_ylabel("y")

        figs.append(fig)
    return figs
