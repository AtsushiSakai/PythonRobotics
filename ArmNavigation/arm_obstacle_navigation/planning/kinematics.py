import torch


def forward_kinematics(
    link_lengths,
    joint_angles,
) -> torch.Tensor:
    """
    Returns:
        joints_pose: N x 2 matrix of joint X,Y coorindates
    """
    joints_pose = torch.zeros((len(joint_angles) + 1, 2))
    theta_sum = torch.tensor(0.0)
    for i, theta in enumerate(joint_angles):
        theta_sum += theta
        xy = torch.tensor(
            [
                torch.cos(theta_sum) * link_lengths[i],
                torch.sin(theta_sum) * link_lengths[i],
            ]
        )
        joints_pose[i + 1, :] = joints_pose[i, :] + xy

    return joints_pose
