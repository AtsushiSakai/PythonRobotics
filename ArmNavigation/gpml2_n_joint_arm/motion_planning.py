"""

GPML2 motion planning for a N-link arm.

This script uses theseus. For installation instructions, visit https://github.com/facebookresearch/theseus#getting-started

This script is based on one of the theseus tutorials: https://github.com/facebookresearch/theseus/blob/main/tutorials/04_motion_planning.ipynb

"""

import os.path
import sys

import theseus as th
import torch
import torch.utils.data
from theseus.optimizer import OptimizerInfo

from utils.collision import CollisionArm
from utils.data import LinkArmDataset, Obstacle
from utils.kinematics import forward_kinematics, inverse_kinematics, resample_trajectory
from utils.viz import generate_figs

torch.set_default_dtype(torch.double)


def create_dataset(expert_trajectory_len: int) -> LinkArmDataset:
    """
    Create a dataset for link arm motion planning.

    Args:
        expert_trajectory_len (int): Length of the expert trajectory.

    Returns:
        LinkArmDataset: The created dataset.

    Raises:
        RuntimeError: If inverse kinematics fails to find a solution.
    """
    link_lengths = torch.tensor([3.0, 2.0, 1.0])

    def create_varying_vars(
        idx: int
    ) -> tuple[torch.Tensor, torch.Tensor, list[Obstacle], torch.Tensor]:
        # Create 2 kinds of data
        if idx % 2 == 0:
            init_joint_angles = torch.tensor([-1.0, -1.0, -1.0])
            target = torch.tensor([4.0, 0.0])
            obstacles = [Obstacle((2.5, -5.5), 1.0, 1.0)]
            midpoint_joint_angles = torch.tensor([-1.0, 0.5, 0.5])
        else:
            init_joint_angles = torch.tensor([-2.0, 1.0, 1.0])
            target = torch.tensor([-4.0, 0.0])
            obstacles = [Obstacle((-2.75, -5.5), 1.0, 1.0)]
            midpoint_joint_angles = torch.tensor([-2.0, -0.5, -0.5])

        # Create expert_trajectory
        target_joint_angles, success = inverse_kinematics(
            link_lengths, midpoint_joint_angles, target
        )
        if not success:
            raise RuntimeError("IK failed!")
        trajectory = torch.stack(
            [init_joint_angles, midpoint_joint_angles, target_joint_angles]
        ).T
        expert_trajectory = resample_trajectory(trajectory, expert_trajectory_len)

        return init_joint_angles, target, obstacles, expert_trajectory

    dataset = LinkArmDataset(
        link_lengths=torch.tensor([3.0, 2.0, 1.0]),
        create_varying_vars=create_varying_vars,
        map_origin=(-9.0, -9.0),
        map_nrows=190,
        map_ncols=190,
        map_cell_size=0.1,
    )
    return dataset


def model_problem(
    nlinks: int,
    trajectory_len: int,
    map_size: int,
    safety_distance: float,
    robot_radius: float,
    total_time: float,
    Qc_inv: list[list[float]],
    collision_w: float,
    boundary_w: float,
) -> th.Objective:
    """
    Creates an optimization problem for arm motion planning.

    Args:
        nlinks (int): Number of links in the arm.
        trajectory_len (int): Length of the trajectory.
        map_size (int): Size of the map.
        safety_distance (float): Safety distance to avoid collisions.
        robot_radius (float): Radius of the robot.
        total_time (float): Total time for the trajectory.
        Qc_inv (list[list[float]]): Inverse of the covariance matrix for GP dynamics cost.
        collision_w (float): Weight for collision avoidance cost.
        boundary_w (float): Weight for boundary cost.

    Returns:
        th.Objective: The optimization objective for arm motion planning.
    """
    num_time_steps = trajectory_len - 1
    dt_val = total_time / num_time_steps

    # =========================
    # Defining Variable objects
    # =========================

    # Create optimization variables
    poses: list[th.Vector] = []
    velocities: list[th.Vector] = []
    poses_xy: dict[int, th.Point2] = {}
    for i in range(trajectory_len):
        poses.append(th.Vector(nlinks, name=f"pose_{i}", dtype=torch.double))
        velocities.append(th.Vector(nlinks, name=f"vel_{i}", dtype=torch.double))
        if i != 0 and i != trajectory_len - 1:
            poses_xy[i] = th.Point2(name=f"pose_xy_{i}")

    # Targets for boundary cost functions
    start = th.Vector(nlinks, name="start")
    goal = th.Point2(name="goal")

    # For collision avoidance cost function
    sdf_origin = th.Point2(name="sdf_origin")
    cell_size = th.Variable(torch.empty(1, 1), name="cell_size")
    sdf_data = th.Variable(torch.empty(1, map_size, map_size), name="sdf_data")
    cost_eps = th.Variable(
        torch.tensor(robot_radius + safety_distance).view(1, 1), name="cost_eps"
    )
    link_lengths = th.Vector(nlinks, name="link_lengths")

    # For GP dynamics cost function
    dt = th.Variable(torch.tensor(dt_val).view(1, 1), name="dt")

    # ============
    # Cost weights
    # ============

    # Cost weight to use for all GP-dynamics cost functions
    gp_cost_weight = th.eb.GPCostWeight(torch.tensor(Qc_inv), dt)

    # Cost weight to use for all collision-avoidance cost functions
    collision_cost_weight = th.ScaleCostWeight(collision_w)

    # For all hard-constraints (end points pos/vel) we use a single scalar weight
    # with high value
    boundary_cost_weight = th.ScaleCostWeight(boundary_w)

    # ==============
    # Cost functions
    # ==============

    objective = th.Objective(dtype=torch.double)

    # -----------------------
    # Boundary cost functions
    # -----------------------

    # Fixed starting position
    objective.add(th.Difference(poses[0], start, boundary_cost_weight, name="pose_0"))

    # Fixed initial velocity
    objective.add(
        th.Difference(
            velocities[0],
            th.Vector(tensor=torch.zeros(1, nlinks)),
            boundary_cost_weight,
            name="vel_0",
        )
    )

    # Fixed goal position
    def pose_N_error(optim_vars, aux_vars):
        """The inverse kinematics error of the final pose in trajectory"""
        (theta,) = optim_vars
        (targeted_pose, link_lengths) = aux_vars
        poses = []
        batch_size = theta.shape[0]
        for i in range(batch_size):
            poses.append(
                forward_kinematics(link_lengths.tensor[i, :], theta.tensor[i, :])[-1]
            )
        pose = th.Point2(tensor=torch.stack(poses))
        return pose.local(targeted_pose)

    objective.add(
        th.AutoDiffCostFunction(
            (poses[-1],),
            pose_N_error,
            2,  # xy_dim
            aux_vars=(goal, link_lengths),
            name="targeted_pose_error_N",
            cost_weight=th.ScaleCostWeight(boundary_w),
            autograd_mode="vmap",
        )
    )

    # Fixed final velocity
    objective.add(
        th.Difference(
            velocities[-1],
            th.Vector(tensor=torch.zeros(1, nlinks)),
            boundary_cost_weight,
            name="vel_N",
        )
    )

    # ------------------------
    # Collision cost functions
    # ------------------------
    def pose_i_error(optim_vars, aux_vars):
        """The inverse kinematics error of the i^th pose in trajectory"""
        (theta, targeted_pose) = optim_vars
        (link_lengths,) = aux_vars
        poses = []
        batch_size = theta.shape[0]
        for i in range(batch_size):
            poses.append(
                forward_kinematics(link_lengths.tensor[i, :], theta.tensor[i, :])[-1]
            )
        pose = th.Point2(tensor=torch.stack(poses))
        return pose.local(targeted_pose)

    for i in range(1, trajectory_len - 1):
        # IK constraints of poses[i] (joint_angles) and poses_xy[i] (target)
        objective.add(
            th.AutoDiffCostFunction(
                (
                    poses[i],
                    poses_xy[i],
                ),
                pose_i_error,
                2,  # xy_dim
                aux_vars=(link_lengths,),
                name=f"targeted_pose_error_{i}",
                cost_weight=th.ScaleCostWeight(boundary_w),
                autograd_mode="vmap",
            )
        )
        # And associated collision costs
        objective.add(
            CollisionArm(
                poses[i],
                link_lengths,
                sdf_origin,
                sdf_data,
                cell_size,
                cost_eps,
                collision_cost_weight,
                name=f"collision_{i}",
            )
        )

    # --------------------------
    # GP-dynamics cost functions
    # --------------------------
    for i in range(1, trajectory_len):
        objective.add(
            th.eb.GPMotionModel(
                poses[i - 1],
                velocities[i - 1],
                poses[i],
                velocities[i],
                dt,
                gp_cost_weight,
                name=f"gp_{i}",
            )
        )

    return objective


def get_straight_line_inputs(
    start: torch.Tensor,
    goal: torch.Tensor,
    link_lengths: torch.Tensor,
    total_time: float,
    trajectory_len: int,
) -> dict:
    """
    Returns:
        A dictionary with pose and velocity variable names associated to a straight line trajectory between start and goal
    """
    start_goal_dist = goal - start
    avg_vel = start_goal_dist / total_time
    unit_trajectory_len = start_goal_dist / (trajectory_len - 1)
    input_dict = {}
    for i in range(trajectory_len):
        input_dict[f"pose_{i}"] = start + unit_trajectory_len * i
        if i == 0 or i == trajectory_len - 1:
            input_dict[f"vel_{i}"] = torch.zeros_like(avg_vel)
        else:
            input_dict[f"vel_{i}"] = avg_vel

        if i != 0 and i != trajectory_len - 1:
            batch_size = start.shape[0]
            input_dict[f"pose_xy_{i}"] = torch.zeros(batch_size, 2)  # 2 is xy_dim
            for j in range(batch_size):
                input_dict[f"pose_xy_{i}"][j] = forward_kinematics(
                    link_lengths[j], input_dict[f"pose_{i}"][j]
                )[-1]

    return input_dict


def run_optimizer(
    batch: dict,
    objective: th.Objective,
    total_time: float,
    trajectory_len: int,
    device: str = "cpu",
) -> tuple[dict[str, torch.Tensor], OptimizerInfo]:
    """
    Runs the optimizer for motion planning.

    Args:
        batch (dict): The input batch containing necessary data for planning.
        objective (th.Objective): The objective function to optimize.
        total_time (float): The total time for the trajectory.
        trajectory_len (int): The length of the trajectory.
        device (str, optional): The device to run the optimizer on. Defaults to "cpu".

    Returns:
        tuple[dict[str, torch.Tensor], OptimizerInfo]: A tuple containing the final values and optimizer information.
    """
    optimizer = th.LevenbergMarquardt(
        objective,
        th.CholeskyDenseSolver,
        max_iterations=50,
        step_size=1.0,
    )
    motion_planner = th.TheseusLayer(optimizer)
    motion_planner.to(device=device, dtype=torch.double)

    start = batch["expert_trajectory"][:, :, 0].to(device)
    goal = batch["target"].to(device)
    link_lengths = batch["link_lengths"].to(device)

    planner_inputs = {
        "sdf_origin": batch["sdf_origin"].to(device),
        "start": start.to(device),
        "goal": goal.to(device),
        "cell_size": batch["cell_size"].to(device),
        "sdf_data": batch["sdf_data"].to(device),
        "link_lengths": batch["link_lengths"].to(device),
    }
    target = batch["expert_trajectory"][:, :, -1].to(device)
    input_dict = get_straight_line_inputs(
        start, target, link_lengths, total_time, trajectory_len
    )
    planner_inputs.update(input_dict)
    with torch.no_grad():
        final_values, info = motion_planner.forward(
            planner_inputs,
            optimizer_kwargs={
                "track_best_solution": True,
                "verbose": True,
                "damping": 0.1,
            },
        )

    return final_values, info


def get_trajectory(values_dict: dict, trajectory_len: int) -> torch.Tensor:
    trajectory = torch.empty(
        values_dict["pose_0"].shape[0],  # batch_size
        values_dict["pose_0"].shape[1],  # nlinks
        trajectory_len,
    )
    for i in range(trajectory_len):
        trajectory[:, :, i] = values_dict[f"pose_{i}"]
    return trajectory


if __name__ == "__main__":
    print(__file__ + " start!!")

    save_figure = "-s" in sys.argv[1:]

    print("Creating a dataset...")
    expert_trajectory_len = 20
    dataset = create_dataset(expert_trajectory_len)
    batch_size = 2
    data_loader = torch.utils.data.DataLoader(dataset, batch_size)

    print("Loading a batch...")
    batch = next(iter(data_loader))
    for k, v in batch.items():
        if k != "id":
            print(f"{k:20s}: {v.shape}", type(v))

    print("Showing/Saving figures...")
    figs = generate_figs(
        link_lengths=batch["link_lengths"],
        target=batch["target"],
        obstacles_tensor=batch["obstacles_tensor"],
        init_joint_angles=batch["init_joint_angles"],
        trajectory=None,
        plot_sdf=True,
        sdf_origin=batch["sdf_origin"],
        sdf_cell_size=batch["cell_size"],
        sdf_data=batch["sdf_data"],
    )
    for i, fig in enumerate(figs):
        fig.suptitle(f"Initial Joint Configuration and Collision Map {i+1}")
        if save_figure:
            print(f"collision_map_{i}.png")
            fig.savefig(
                os.path.join(os.path.dirname(__file__), f"collision_map_{i}.png")
            )
        else:
            fig.show()

    print("Setting up an objective...")
    nlinks = batch["link_lengths"].shape[1]
    trajectory_len = batch["expert_trajectory"].shape[2]
    map_size = batch["map_tensor"].shape[1]
    safety_distance = 0.4
    robot_radius = 0.4
    total_time = 10.0
    objective = model_problem(
        nlinks=nlinks,
        trajectory_len=trajectory_len,
        map_size=map_size,
        safety_distance=safety_distance,
        robot_radius=robot_radius,
        total_time=total_time,
        Qc_inv=[[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]],
        collision_w=100.0,
        boundary_w=100.0,
    )

    print("Start running optimizer...")
    final_values, info = run_optimizer(batch, objective, total_time, trajectory_len)
    if info.best_solution is None:
        raise ValueError("Error!")

    print("Showing/Saving result trajectories...")
    result_trajectory = get_trajectory(info.best_solution, trajectory_len)
    figs = generate_figs(
        link_lengths=batch["link_lengths"],
        target=batch["target"],
        obstacles_tensor=batch["obstacles_tensor"],
        trajectory=result_trajectory,
    )
    for i, fig in enumerate(figs):
        fig.suptitle(f"Planned Trajectory {i+1}")
        if save_figure:
            print(i, f"result{i}.png")
            fig.savefig(os.path.join(os.path.dirname(__file__), f"result_{i}.png"))
        else:
            fig.show()
    if not save_figure:
        input("Press Enter to exit...")

    print(__file__ + " done!!")
