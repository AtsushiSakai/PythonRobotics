import matplotlib.pyplot as plt
import numpy as np
import theseus as th
import torch
from matplotlib.axes import Axes
from matplotlib.figure import Figure

from .kinematics import forward_kinematics


def draw_robot(ax: Axes, joints_xy: torch.Tensor, alpha: float = 1.0) -> None:
    """
    Draw the robot arm on the given Axes object.

    Args:
        ax (Axes): The Axes object to draw on.
        joints_xy (torch.Tensor): The joint positions of the robot arm as a 2D tensor.
        alpha (float): The transparency of the robot arm and joint markers.
    """
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
    init_joint_angles: torch.Tensor | None = None,
    trajectory: torch.Tensor | None = None,
    plot_sdf: bool = False,
    sdf_origin: torch.Tensor | None = None,
    sdf_cell_size: torch.Tensor | None = None,
    sdf_data: torch.Tensor | None = None,
) -> list[Figure]:
    """
    Generate figures for visualizing arm navigation.

    Args:
        link_lengths (torch.Tensor): Tensor containing the lengths of the arm links.
        target (torch.Tensor): Tensor containing the target position.
        obstacles_tensor (torch.Tensor): Tensor containing the obstacles.
        init_joint_angles (torch.Tensor | None, optional): Tensor containing the initial joint angles. Defaults to None.
        trajectory (torch.Tensor | None, optional): Tensor containing the arm trajectory. Defaults to None.
        plot_sdf (bool, optional): Flag indicating whether to plot the signed distance field. Defaults to False.
        sdf_origin (torch.Tensor | None, optional): Tensor containing the origin of the signed distance field. Defaults to None.
        sdf_cell_size (torch.Tensor | None, optional): Tensor containing the cell size of the signed distance field. Defaults to None.
        sdf_data (torch.Tensor | None, optional): Tensor containing the data of the signed distance field. Defaults to None.

    Returns:
        list[Figure]: List of generated figures.
    """
    figs: list[Figure] = []
    batch_size = link_lengths.shape[0]
    for i in range(batch_size):
        fig, axs = plt.subplots(1, 2 if plot_sdf else 1)
        figs.append(fig)

        path_ax = axs if not plot_sdf else axs[0]

        for obstacle in obstacles_tensor[i]:
            path_ax.plot(obstacle[:, 0], obstacle[:, 1], "k-")

        path_ax.plot(target[i, 0], target[i, 1], "go")

        if trajectory is not None:
            for j in range(trajectory.shape[2]):
                cur_trajectory = trajectory[i, :, j]
                joints_xy = forward_kinematics(link_lengths[i], cur_trajectory)
                alpha = j / (trajectory.shape[2] - 1)
                draw_robot(path_ax, joints_xy, alpha)

        if init_joint_angles is not None:
            joints_xy = forward_kinematics(link_lengths[i], init_joint_angles[i])
            draw_robot(path_ax, joints_xy, 1.0)

        lim_val = torch.sum(link_lengths[i]) * 1.5
        path_ax.set(xlim=(-lim_val, lim_val), ylim=(-lim_val, lim_val))
        path_ax.grid(visible=True)
        path_ax.set_xlabel("x")
        path_ax.set_ylabel("y")

        if plot_sdf:
            if sdf_origin is None or sdf_cell_size is None or sdf_data is None:
                raise ValueError(
                    "sdf_origin or sdf_cell_size or sdf_data must not be None"
                )
            sdf = th.eb.SignedDistanceField2D(sdf_origin, sdf_cell_size, sdf_data)
            im = axs[1].imshow(
                np.flipud(sdf.sdf_data.tensor[i].cpu().numpy()), cmap="plasma_r"
            )
            fig.colorbar(im)
    return figs
