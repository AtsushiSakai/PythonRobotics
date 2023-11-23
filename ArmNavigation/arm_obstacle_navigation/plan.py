import sys
import warnings

import theseus as th
import torch
import torch.utils.data
from theseus.optimizer import OptimizerInfo
from planning.data import LinkArmDataset, Obstacle
from planning.kinematics import inverse_kinematics, resample_trajectory
from planning.viz import generate_figs
from planning.collision import CollisionArm

torch.set_default_dtype(torch.double)


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
        ).T
        expert_trajectory = resample_trajectory(trajectory, trajectory_len)

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
    joints: list[th.Vector] = []
    velocities: list[th.Vector] = []
    for i in range(trajectory_len):
        joints.append(th.Vector(num_links, name=f"joint_{i}", dtype=torch.double))
        velocities.append(th.Vector(num_links, name=f"vel_{i}", dtype=torch.double))

    # Targets for joint boundary cost functions
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
    objective.add(th.Difference(joints[0], start, boundary_cost_weight, name="joint_0"))

    # Fixed initial velocity
    objective.add(
        th.Difference(
            velocities[0],
            th.Vector(tensor=torch.zeros(1, num_links)),
            boundary_cost_weight,
            name="vel_0",
        )
    )
    objective.add(th.Difference(joints[-1], goal, boundary_cost_weight, name="joint_N"))
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
                joints[i],
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
                joints[i - 1],
                velocities[i - 1],
                joints[i],
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
    # Returns a dictionary with pose and velocity variable names associated to a
    # straight line trajectory between start and goal
    start_goal_dist = goal - start
    avg_vel = start_goal_dist / total_time
    unit_trajectory_len = start_goal_dist / (trajectory_len - 1)
    input_dict = {}
    for i in range(trajectory_len):
        input_dict[f"joint_{i}"] = start + unit_trajectory_len * i
        if i == 0 or i == trajectory_len - 1:
            input_dict[f"vel_{i}"] = torch.zeros_like(avg_vel)
        else:
            input_dict[f"vel_{i}"] = avg_vel
    return input_dict


if __name__ == "__main__":
    debug = "-d" in sys.argv[1:]

    # Create a dataset and read the first one
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

    # Set up the problem
    num_links = batch["link_lengths"].shape[1]
    trajectory_len = batch["expert_trajectory"].shape[2]
    print("trajectory_len", trajectory_len)
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
    print("objective=", objective)

    print("Done!")
