import sys

import theseus as th
import torch
import torch.utils.data
from utils.collision import CollisionArm
from utils.data import LinkArmDataset, Obstacle
from utils.kinematics import inverse_kinematics, resample_trajectory
from utils.viz import generate_figs
from theseus.optimizer import OptimizerInfo

torch.set_default_dtype(torch.double)


def create_dataset(expert_trajectory_len: int) -> LinkArmDataset:
    link_lengths = torch.tensor([3.0, 2.0, 1.0])

    def create_varying_vars(
        idx: int
    ) -> tuple[torch.Tensor, torch.Tensor, list[Obstacle], torch.Tensor]:
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
        map_num_rows=180,
        map_num_cols=180,
        map_cell_size=0.1,
    )
    return dataset


def model_problem(
    num_links: int,
    trajectory_len: int,
    map_size: int,
    safety_distance: float,
    robot_radius: float,
    total_time: float,
    Qc_inv: list[list[float]],
    collision_w: float,
    boundary_w: float,
) -> th.Objective:
    num_time_steps = trajectory_len - 1
    dt_val = total_time / num_time_steps

    # =========================
    # Defining Variable objects
    # =========================

    # Create optimization variables
    poses: list[th.Vector] = []
    velocities: list[th.Vector] = []
    for i in range(trajectory_len):
        poses.append(th.Vector(num_links, name=f"pose_{i}", dtype=torch.double))
        velocities.append(th.Vector(num_links, name=f"vel_{i}", dtype=torch.double))

    # Targets for boundary cost functions
    start = th.Vector(num_links, name="start")
    goal = th.Vector(num_links, name="goal")

    # For collision avoidance cost function
    sdf_origin = th.Point2(name="sdf_origin")
    cell_size = th.Variable(torch.empty(1, 1), name="cell_size")
    sdf_data = th.Variable(torch.empty(1, map_size, map_size), name="sdf_data")
    cost_eps = th.Variable(
        torch.tensor(robot_radius + safety_distance).view(1, 1), name="cost_eps"
    )
    link_lengths = th.Vector(num_links, name="link_lengths")
    init_joint_angles = th.Vector(num_links, name="init_joint_angles")

    # For GP dynamics cost function
    dt = th.Variable(torch.tensor(dt_val).view(1, 1), name="dt")

    # ============
    # Cost weights
    # ============

    # Cost weight to use for all GP-dynamics cost functions
    gp_cost_weight = th.eb.GPCostWeight(torch.tensor(Qc_inv), dt)

    # Cost weight to use for all collision-avoidance cost functions
    collision_cost_weight = th.ScaleCostWeight(th.Variable(torch.tensor(collision_w)))

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
            th.Vector(tensor=torch.zeros(1, num_links)),
            boundary_cost_weight,
            name="vel_0",
        )
    )

    # Fixed goal position
    objective.add(th.Difference(poses[-1], goal, boundary_cost_weight, name="pose_N"))

    # Fixed final velocity
    objective.add(
        th.Difference(
            velocities[-1],
            th.Vector(tensor=torch.zeros(1, num_links)),
            boundary_cost_weight,
            name="vel_N",
        )
    )

    # ------------------------
    # Collision cost functions
    # ------------------------
    for i in range(1, trajectory_len - 1):
        objective.add(
            CollisionArm(
                poses[i],
                link_lengths,
                init_joint_angles,
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
    start: torch.Tensor, goal: torch.Tensor, total_time: float, trajectory_len: int
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
    return input_dict


def run_optimizer(
    batch: dict,
    objective: th.Objective,
    total_time: float,
    trajectory_len: int,
    device: str = "cpu",
) -> tuple[dict[str, torch.Tensor], OptimizerInfo]:
    optimizer = th.LevenbergMarquardt(
        objective,
        th.CholeskyDenseSolver,
        max_iterations=50,
        step_size=1.0,
    )
    motion_planner = th.TheseusLayer(optimizer)
    motion_planner.to(device=device, dtype=torch.double)

    start = batch["expert_trajectory"][:, :, 0].to(device)
    goal = batch["expert_trajectory"][:, :, -1].to(device)

    planner_inputs = {
        "sdf_origin": batch["sdf_origin"].to(device),
        "start": start.to(device),
        "goal": goal.to(device),
        "cell_size": batch["cell_size"].to(device),
        "sdf_data": batch["sdf_data"].to(device),
        "link_lengths": batch["link_lengths"].to(device),
        "init_joint_angles": batch["init_joint_angles"].to(device),
    }
    input_dict = get_straight_line_inputs(start, goal, total_time, trajectory_len)
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


def get_trajectory(
    values_dict: dict, trajectory_len: int, device: str = "cpu"
) -> torch.Tensor:
    trajectory = torch.empty(
        values_dict["pose_0"].shape[0],  # batch_size
        values_dict["pose_0"].shape[1],  # ndim
        trajectory_len,
        device=device,
    )
    for i in range(trajectory_len):
        trajectory[:, :, i] = values_dict[f"pose_{i}"]
    return trajectory


if __name__ == "__main__":
    print(__file__ + " start!!")

    debug = "-d" in sys.argv[1:]

    print("Creating a dataset...")
    expert_trajectory_len = 10
    dataset = create_dataset(expert_trajectory_len)
    batch_size = 2
    data_loader = torch.utils.data.DataLoader(dataset, batch_size)

    print("Loading a batch...")
    batch = next(iter(data_loader))
    if debug:
        for k, v in batch.items():
            if k != "id":
                print(f"{k:20s}: {v.shape}", type(v))

        print("Saving figures...")
        figs = generate_figs(
            link_lengths=batch["link_lengths"],
            init_joint_angles=batch["init_joint_angles"],
            target=batch["target"],
            obstacles_tensor=batch["obstacles_tensor"],
            trajectory=batch["expert_trajectory"],
        )
        for i, fig in enumerate(figs):
            fig.savefig(f"expert_trajectory_{0}.png".format(i))
            fig.savefig(f"expert_trajectory_{0}.png".format(i))

    print("Setting up an objective...")
    num_links = batch["link_lengths"].shape[1]
    trajectory_len = batch["expert_trajectory"].shape[2]
    map_size = batch["map_tensor"].shape[1]
    robot_radius = 0.4
    total_time = 10.0
    objective = model_problem(
        num_links=num_links,
        trajectory_len=trajectory_len,
        map_size=map_size,
        safety_distance=0.4,
        robot_radius=robot_radius,
        total_time=total_time,
        Qc_inv=[[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]],
        collision_w=20.0,
        boundary_w=100.0,
    )

    print("Start running optimizer...")
    final_values, info = run_optimizer(batch, objective, total_time, trajectory_len)
    if info.best_solution is None:
        raise ValueError("Error!")

    print("Saving result trajectories...")
    result_trajectory = get_trajectory(info.best_solution, trajectory_len)
    figs = generate_figs(
        link_lengths=batch["link_lengths"],
        init_joint_angles=batch["init_joint_angles"],
        target=batch["target"],
        obstacles_tensor=batch["obstacles_tensor"],
        trajectory=result_trajectory,
    )
    figs[0].savefig("restrajectory0.png")
    figs[1].savefig("restrajectory1.png")

    print(__file__ + " done!!")
