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
        sdf_origin: Point2 | torch.Tensor,
        sdf_data: torch.Tensor | Variable,
        sdf_cell_size: float | torch.Tensor | Variable,
        cost_eps: float | Variable | torch.Tensor,
        cost_weight: CostWeight,
        name: str | None = None,
    ):
        """
        Represents a collision cost function for an arm.

        Args:
            pose (Vector): The pose of the arm.
            link_lengths (Vector): The lengths of the arm links.
            sdf_origin (Point2 | torch.Tensor): The origin of the signed distance field.
            sdf_data (torch.Tensor | Variable): The data of the signed distance field.
            sdf_cell_size (float | torch.Tensor | Variable): The cell size of the signed distance field.
            cost_eps (float | Variable | torch.Tensor): The cost epsilon value.
            cost_weight (CostWeight): The weight of the cost function.
            name (str | None, optional): The name of the collision cost function. Defaults to None.
        """
        super().__init__(cost_weight, name=name)
        self.pose = pose
        self.link_lengths = link_lengths
        self.sdf_origin = SignedDistanceField2D.convert_origin(sdf_origin)
        self.sdf_data = SignedDistanceField2D.convert_sdf_data(sdf_data)
        self.sdf_cell_size = SignedDistanceField2D.convert_cell_size(sdf_cell_size)
        self.cost_eps = as_variable(cost_eps)
        self.cost_eps.tensor = self.cost_eps.tensor.view(-1, 1)

        self.register_optim_vars(["pose"])
        self.register_aux_vars(
            [
                "link_lengths",
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
        """
        Computes the distances and jacobians for the arm.

        Returns:
            tuple[torch.Tensor, torch.Tensor]: A tuple containing the distances and jacobians.
        """
        batch_size, xy_ndim = self.sdf_origin.tensor.shape
        nlinks = self.link_lengths.shape[1]
        poses = torch.zeros(batch_size, xy_ndim)
        jac_pose = torch.zeros(batch_size, xy_ndim, nlinks)
        for i in range(batch_size):
            poses[i] = forward_kinematics(
                link_lengths=self.link_lengths.tensor[i],
                joint_angles=self.pose.tensor[i],
            )[-1]
            jac_pose[i] = jacobian(
                link_lengths=self.link_lengths.tensor[i],
                joint_angles=self.pose.tensor[i],
            )

        dist, jac = self.sdf.signed_distance(poses.view(-1, xy_ndim, 1))
        jac_out = jac.matmul(jac_pose)
        return dist, jac_out

    def _error_from_distances(self, distances: torch.Tensor):
        """
        Computes the error from the distances.

        Args:
            distances (torch.Tensor): The distances.

        Returns:
            torch.Tensor: The error.
        """
        return (self.cost_eps.tensor - distances).clamp(min=0)

    def error(self) -> torch.Tensor:
        """
        Computes the error of the collision cost function.

        Returns:
            torch.Tensor: The error.
        """
        distances, _ = self._compute_distances_and_jacobians()
        return self._error_from_distances(distances)

    def jacobians(self) -> tuple[list[torch.Tensor], torch.Tensor]:
        """
        Computes the jacobians of the collision cost function.

        Returns:
            tuple[list[torch.Tensor], torch.Tensor]: A tuple containing the jacobians and the error.
        """
        distances, jacobian = self._compute_distances_and_jacobians()
        error = self._error_from_distances(distances)
        faraway_idx = distances > self.cost_eps.tensor
        jacobian[faraway_idx] = 0.0
        return [-jacobian], error

    def _copy_impl(self, new_name: str | None = None) -> "CollisionArm":
        """
        Creates a copy of the collision cost function.

        Args:
            new_name (str | None, optional): The name of the new collision cost function. Defaults to None.

        Returns:
            CollisionArm: The copied collision cost function.
        """
        return CollisionArm(
            self.pose.copy(),
            self.link_lengths.copy(),
            self.sdf_origin.copy(),
            self.sdf_data.copy(),
            self.sdf_cell_size.copy(),
            self.cost_eps.copy(),
            self.weight.copy(),
            name=new_name,
        )

    def dim(self) -> int:
        """
        Returns the dimension of the collision cost function.

        Returns:
            int: The dimension.
        """
        return 1

    def set_aux_var_at(self, index: int, variable: Variable):
        """
        Sets the auxiliary variable at the specified index.

        Args:
            index (int): The index of the auxiliary variable.
            variable (Variable): The auxiliary variable.

        Note:
            This function is needed so that the SDF container also updates with the new aux variables.
        """
        super().set_aux_var_at(index, variable)
        self.sdf.update_data(self.sdf_origin, self.sdf_data, self.sdf_cell_size)
