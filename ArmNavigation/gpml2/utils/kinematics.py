import numpy as np
import torch
from scipy.interpolate import interp1d

xy_ndim = 2  # pose space ndim


def forward_kinematics(
    link_lengths: torch.Tensor,
    joint_angles: torch.Tensor,
    device: str = "cpu",
    dtype: torch.dtype = torch.double,
) -> torch.Tensor:
    """
    Calculates the forward kinematics of a chain of links and joints.

    Args:
        link_lengths (torch.Tensor): A tensor containing the lengths of each arm link.
        joint_angles (torch.Tensor): A tensor containing the angles of each joint.
        device (str, optional): The device to perform the calculations on. Defaults to "cpu".
        dtype (torch.dtype, optional): The data type to use for the calculations. Defaults to torch.double.

    Returns:
        torch.Tensor: A tensor containing the x, y coordinates of each joint.
    """
    joints_xy = torch.zeros(len(joint_angles) + 1, 2)
    joints_xy.to(device, dtype)

    theta_sum = torch.zeros_like(joint_angles[0])
    for i, theta in enumerate(joint_angles):
        theta_sum = theta_sum + theta  # Avoid in-place modification
        xy = torch.stack(
            [
                torch.cos(theta_sum) * link_lengths[i],
                torch.sin(theta_sum) * link_lengths[i],
            ]
        )
        joints_xy = torch.cat(
            [joints_xy[: i + 1, :], (joints_xy[i, :] + xy).unsqueeze(0)], dim=0
        )

    return joints_xy


def jacobian(
    link_lengths: torch.Tensor,
    joint_angles: torch.Tensor,
) -> torch.Tensor:
    """Compute the Jacobian matrix for a robotic arm.

    Args:
        link_lengths (torch.Tensor): Tensor containing the lengths of the arm links.
        joint_angles (torch.Tensor): Tensor containing the joint angles of the arm.

    Returns:
        torch.Tensor: The Jacobian matrix.
    """
    joints_xy = forward_kinematics(link_lengths, joint_angles)
    ee_xy = joints_xy[-1, :]
    jac = torch.zeros((2, len(joint_angles)))
    for i, joint_xy in enumerate(joints_xy[:-1]):
        delta_xy = ee_xy - joint_xy
        delta_ang = torch.atan2(delta_xy[1], delta_xy[0])
        delta_dist = torch.norm(delta_xy)

        dx = -torch.sin(delta_ang) * delta_dist
        dy = torch.cos(delta_ang) * delta_dist
        jac[0, i] = dx
        jac[1, i] = dy

    return jac


def inverse_kinematics(
    link_lengths: torch.Tensor,
    joint_angles: torch.Tensor,
    target: torch.Tensor,
    max_iteration: int = 1000,
    damping: float = 0.1,
) -> tuple[torch.Tensor, bool]:
    """
    Calculates the inverse kinematics of a robotic arm.

    Args:
        link_lengths (torch.Tensor): Tensor containing the lengths of each link in the arm.
        joint_angles (torch.Tensor): Tensor containing the initial joint angles of the arm.
        target (torch.Tensor): Tensor containing the target position for the end effector.
        max_iteration (int, optional): Maximum number of iterations for the inverse kinematics solver. Defaults to 1000.
        damping (float, optional): Damping factor for the solver. Defaults to 0.1.

    Returns:
        tuple[torch.Tensor, bool]: A tuple containing the final joint angles and a boolean indicating whether the solver converged.
    """
    damping_tensor = torch.tensor(damping)

    cur_joint_ang = joint_angles.clone()
    iter = 0
    while iter < max_iteration:
        cur_xy = forward_kinematics(link_lengths, cur_joint_ang)[-1, :]
        delta_xy = target - cur_xy
        dist = torch.norm(delta_xy)
        if dist.item() < 0.01:
            return cur_joint_ang, True

        delta_ang = (
            torch.pinverse(jacobian(link_lengths, cur_joint_ang))
            @ (target - cur_xy)
            * damping_tensor
        )
        cur_joint_ang = cur_joint_ang + delta_ang
        iter += 1

    return cur_joint_ang, False


def resample_trajectory(trajectory: torch.Tensor, num_points: int) -> torch.Tensor:
    """
    Resamples a trajectory to have a specified number of points.

    Args:
        trajectory (torch.Tensor): The input trajectory tensor.
        num_points (int): The desired number of points in the resampled trajectory.

    Returns:
        torch.Tensor: The resampled trajectory tensor.

    Raises:
        ValueError: If the input trajectory has less than or equal to one point.
    """
    if trajectory.size(1) <= 1:
        raise ValueError("Trajectory must have more than one point.")

    trajectory_np = trajectory.numpy()
    current_indices = np.linspace(0, 1, trajectory_np.shape[1])
    new_indices = np.linspace(0, 1, num_points)
    interpolator = interp1d(current_indices, trajectory_np, axis=1, kind="linear")
    new_trajectory_np = interpolator(new_indices)

    new_trajectory = torch.from_numpy(new_trajectory_np)
    return new_trajectory
