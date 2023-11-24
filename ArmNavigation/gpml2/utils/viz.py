import matplotlib.pyplot as plt
import torch
from matplotlib.axes import Axes
from matplotlib.figure import Figure
from .kinematics import forward_kinematics


def draw_robot(
    ax: Axes,
    joints_xy: torch.Tensor,
    alpha: float = 1.0
) -> None:
    ax.plot(joints_xy[:, 0], joints_xy[:, 1], "r-", alpha=alpha)
    ax.plot(
        joints_xy[:, 0],
        joints_xy[:, 1],
        "ko",
        alpha=alpha,
    )


def generate_figs(
    link_lengths: torch.Tensor,
    target: torch.Tensor,
    obstacles_tensor: torch.Tensor,
    trajectory: torch.Tensor,
) -> list[Figure]:
    figs: list[Figure] = []
    batch_size = link_lengths.shape[0]
    for i in range(batch_size):
        fig, axs = plt.subplots(1, 1)

        for j in range(trajectory.shape[2]):
            cur_trajectory = trajectory[i, :, j]
            joints_xy = forward_kinematics(link_lengths[i], cur_trajectory)
            alpha = j / (trajectory.shape[2] - 1)
            draw_robot(axs, joints_xy, alpha)

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
