import matplotlib.pyplot as plt
import numpy as np
import theseus as th
import torch
from matplotlib.axes import Axes
from matplotlib.figure import Figure

from .kinematics import forward_kinematics


def draw_robot(ax: Axes, joints_xy: torch.Tensor, alpha: float = 1.0) -> None:
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
    plot_sdf: bool = False,
    sdf_origin: torch.Tensor | None = None,
    sdf_cell_size: torch.Tensor | None = None,
    sdf_data: torch.Tensor | None = None,
) -> list[Figure]:
    figs: list[Figure] = []
    batch_size = link_lengths.shape[0]
    for i in range(batch_size):
        fig, axs = plt.subplots(1, 2 if plot_sdf else 1)
        figs.append(fig)

        path_ax = axs if not plot_sdf else axs[0]

        for j in range(trajectory.shape[2]):
            cur_trajectory = trajectory[i, :, j]
            joints_xy = forward_kinematics(link_lengths[i], cur_trajectory)
            alpha = j / (trajectory.shape[2] - 1)
            draw_robot(path_ax, joints_xy, alpha)

        for obstacle in obstacles_tensor[i]:
            path_ax.plot(obstacle[:, 0], obstacle[:, 1], "k-")

        path_ax.plot(target[i, 0], target[i, 1], "go")

        lim_val = torch.sum(link_lengths[i]) * 1.5
        path_ax.set(xlim=(-lim_val, lim_val), ylim=(-lim_val, lim_val))
        path_ax.grid(visible=True)
        path_ax.set_xlabel("x")
        path_ax.set_ylabel("y")

        if not plot_sdf:
            continue
        if sdf_origin is None or sdf_cell_size is None or sdf_data is None:
            raise ValueError("sdf_origin or sdf_cell_size or sdf_data must not be None")
        sdf = th.eb.SignedDistanceField2D(sdf_origin, sdf_cell_size, sdf_data)
        im = axs[1].imshow(
            np.flipud(sdf.sdf_data.tensor[i].cpu().numpy()), cmap="plasma_r"
        )
        fig.colorbar(im)
    return figs
