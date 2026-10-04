import conftest  # Add root path to sys.path
import matplotlib.pyplot as plt
from matplotlib.animation import PillowWriter
import numpy as np
from numpy.testing import assert_allclose
import pytest

from PathPlanning.Flocking import flocking as m


def test_sigma_norm_and_gradient_at_zero():
    assert m.sigma_norm([0.0, 0.0]) == 0.0
    assert_allclose(m.sigma_norm([3.0, 4.0]),
                    (np.sqrt(1.0 + 25.0 * m.EPSILON) - 1.0) / m.EPSILON)
    assert m.sigma_norm([1e-10, 0.0]) > 0.0
    gradient, alignment = m.interaction_terms(np.zeros((2, 2)), np.zeros((2, 2)))
    assert_allclose(gradient, 0.0)
    assert_allclose(alignment, 0.0)


def test_bump_cutoff_and_transition():
    values = m.bump_function(np.array([-0.1, 0.0, m.BUMP_H,
                                       (1.0 + m.BUMP_H) / 2, 1.0, 1.1]))
    assert_allclose(values, [0.0, 1.0, 1.0, 0.5, 0.0, 0.0], atol=1e-15)


@pytest.mark.parametrize("distance, sign", [(2.5, -1), (5.0, 0), (5.5, 1),
                                           (6.0, 0), (7.0, 0)])
def test_pair_repulsion_equilibrium_attraction_and_cutoff(distance, sign):
    positions = np.array([[0.0, 0.0], [distance, 0.0]])
    gradient, _ = m.interaction_terms(positions, np.zeros((2, 2)))
    assert np.sign(gradient[0, 0]) == sign
    assert_allclose(gradient[0], -gradient[1])
    assert_allclose(gradient[:, 1], 0.0)


def test_alignment_dissipates_relative_velocity_and_has_finite_range():
    positions = np.array([[0.0, 0.0], [m.DESIRED_DISTANCE, 0.0]])
    velocities = np.array([[2.0, -1.0], [-1.0, 3.0]])
    _, alignment = m.interaction_terms(positions, velocities)
    assert_allclose(alignment.sum(axis=0), 0.0)
    assert np.sum(velocities * alignment) < 0.0
    _, matched = m.interaction_terms(positions, np.ones((2, 2)))
    assert_allclose(matched, 0.0)
    positions[1, 0] = m.INTERACTION_RANGE
    _, disconnected = m.interaction_terms(positions, velocities)
    assert_allclose(disconnected, 0.0)


def test_centroid_acceleration_depends_only_on_navigation():
    rng = np.random.default_rng(5)
    positions = rng.uniform(-5.0, 5.0, (10, 2))
    velocities = rng.normal(size=(10, 2))
    reference, reference_velocity = np.array([7.0, 3.0]), np.array([1.0, 0.5])
    acceleration = m.flocking_control(positions, velocities, reference, reference_velocity)
    expected = (-m.POSITION_GAIN * (positions.mean(axis=0) - reference)
                - m.VELOCITY_GAIN * (velocities.mean(axis=0) - reference_velocity))
    assert_allclose(acceleration.mean(axis=0), expected, atol=1e-14)
    # With one agent, the navigation feedback is the entire controller.
    assert_allclose(m.flocking_control(positions[:1], velocities[:1],
                                      positions[0], velocities[0]), 0.0)


def test_controller_is_invariant_to_agent_order_and_coordinate_frame():
    rng = np.random.default_rng(4)
    q, p = rng.normal(size=(2, 6, 2))
    qr, pr = np.array([5.0, 2.0]), np.array([1.0, 0.5])
    rotation = np.array([[0.0, -1.0], [1.0, 0.0]])
    translation, boost = np.array([4.0, -8.0]), np.array([-2.0, 3.0])
    order = np.array([3, 0, 5, 2, 4, 1])
    expected = m.flocking_control(q, p, qr, pr)[order] @ rotation
    transformed = m.flocking_control(q[order] @ rotation + translation,
                                    p[order] @ rotation + boost,
                                    qr @ rotation + translation, pr @ rotation + boost)
    assert_allclose(transformed, expected, atol=1e-13)


@pytest.mark.parametrize("seed", [0, 1, 2])
def test_flock_converges_without_collisions_for_demo_initial_conditions(seed):
    times, q, p, references = m.simulate(seed=seed)
    assert np.isfinite(q).all() and np.isfinite(p).all()
    velocity_error = np.sqrt(np.mean(np.sum((p[-1] - [1.0, 0.5]) ** 2, axis=1)))
    assert velocity_error < 0.01
    assert np.linalg.norm(q[-1].mean(axis=0) - references[-1]) < 0.01
    distance = np.linalg.norm(q[:, :, None, :] - q[:, None, :, :], axis=-1)
    distance[:, np.arange(25), np.arange(25)] = np.inf
    assert distance.min() > 3.0
    adjacency = (distance[-1] < m.INTERACTION_RANGE).astype(float)
    laplacian = np.diag(adjacency.sum(axis=1)) - adjacency
    assert np.linalg.eigvalsh(laplacian)[1] > 0.1  # One connected flock.

    # Independent closed-form solution for critically damped centroid motion.
    omega = np.sqrt(m.POSITION_GAIN)
    e0 = q[0].mean(axis=0) - references[0]
    v0 = p[0].mean(axis=0) - [1.0, 0.5]
    t = times[:, None]
    expected_error = (e0 + (v0 + omega * e0) * t) * np.exp(-omega * t)
    assert_allclose(q.mean(axis=1) - references, expected_error, atol=0.025)


def test_animation_can_be_rendered(tmp_path):
    animation = m.create_animation(*m.simulate(simulation_time=0.1))
    output = tmp_path / "flocking.gif"
    animation.save(output, writer=PillowWriter(fps=10))
    assert output.stat().st_size > 0
    plt.close("all")


def test_main_without_animation(monkeypatch):
    monkeypatch.setattr(m, "show_animation", False)
    m.main()


if __name__ == "__main__":
    conftest.run_this_test(__file__)
