import torch
from theseus.core import CostFunction, CostWeight, Variable, as_variable
from theseus.embodied.collision.signed_distance_field import SignedDistanceField2D
from theseus.geometry import Point2, Vector

from .kinematics import forward_kinematics, jacobian


class CollisionArm(CostFunction):
    def __init__(
        self,
        pose: Vector,
        link_lengths: Vector,
        init_joint_angles: Vector,
        sdf_origin: Point2 | torch.Tensor,
        sdf_data: torch.Tensor | Variable,
        sdf_cell_size: float | torch.Tensor | Variable,
        cost_eps: float | Variable | torch.Tensor,
        cost_weight: CostWeight,
        name: str | None = None,
    ):
        super().__init__(cost_weight, name=name)
        self.pose = pose
        self.link_lengths = link_lengths
        self.init_joint_angles = init_joint_angles
        self.sdf_origin = SignedDistanceField2D.convert_origin(sdf_origin)
        self.sdf_data = SignedDistanceField2D.convert_sdf_data(sdf_data)
        self.sdf_cell_size = SignedDistanceField2D.convert_cell_size(sdf_cell_size)
        self.cost_eps = as_variable(cost_eps)
        self.cost_eps.tensor = self.cost_eps.tensor.view(-1, 1)

        self.register_optim_vars(["pose"])
        self.register_aux_vars(
            [
                "link_lengths",
                "init_joint_angles",
                "sdf_origin",
                "sdf_data",
                "sdf_cell_size",
                "cost_eps",
            ]
        )
        self.sdf = SignedDistanceField2D(
            self.sdf_origin, self.sdf_cell_size, self.sdf_data
        )

    def _compute_distances_and_jacobians(
        self,
    ) -> tuple[torch.Tensor, torch.Tensor]:
        robot_state = torch.zeros_like(self.sdf_origin.tensor)
        jac_pose = torch.zeros(
            self.sdf_origin.tensor.shape[0],
            self.sdf_origin.tensor.shape[1],
            self.init_joint_angles.tensor.shape[1],
        )

        for i in range(self.pose.shape[0]):
            pose = forward_kinematics(
                link_lengths=self.link_lengths[i],
                joint_angles=self.init_joint_angles[i],
            )[-1]
            robot_state[i] = pose
            j = jacobian(
                link_lengths=self.link_lengths[i],
                joint_angles=self.init_joint_angles[i],
            )
            jac_pose[i] = j

        dist, jac = self.sdf.signed_distance(robot_state.view(-1, 2, 1))
        if jac_pose is not None:
            jac = jac.matmul(jac_pose)
        return dist, jac

    def _error_from_distances(self, distances: torch.Tensor):
        return (self.cost_eps.tensor - distances).clamp(min=0)

    def error(self) -> torch.Tensor:
        distances, _ = self._compute_distances_and_jacobians()
        return self._error_from_distances(distances)

    def jacobians(self) -> tuple[list[torch.Tensor], torch.Tensor]:
        distances, jacobian = self._compute_distances_and_jacobians()
        error = self._error_from_distances(distances)
        faraway_idx = distances > self.cost_eps.tensor
        jacobian[faraway_idx] = 0.0
        return [-jacobian], error

    def _copy_impl(self, new_name: str | None = None) -> "CollisionArm":
        return CollisionArm(
            self.pose.copy(),
            self.link_lengths.copy(),
            self.init_joint_angles.copy(),
            self.sdf_origin.copy(),
            self.sdf_data.copy(),
            self.sdf_cell_size.copy(),
            self.cost_eps.copy(),
            self.weight.copy(),
            name=new_name,
        )

    def dim(self) -> int:
        return 1

    # This is needed so that the SDF container also updates with the new aux var
    def set_aux_var_at(self, index: int, variable: Variable):
        super().set_aux_var_at(index, variable)
        self.sdf.update_data(self.sdf_origin, self.sdf_data, self.sdf_cell_size)
