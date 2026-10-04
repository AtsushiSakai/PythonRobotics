"""Olfati-Saber's Algorithm 2 for flocking in an obstacle-free plane.

Run this file to animate 25 double-integrator agents following a moving
reference. Arrows show velocities; lines connect neighbors within range.

Reference: R. Olfati-Saber, Flocking for Multi-Agent Dynamic Systems:
Algorithms and Theory, IEEE TAC 51(3), 401-420, 2006.
https://doi.org/10.1109/TAC.2005.864190
"""

import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from matplotlib.collections import LineCollection
import numpy as np

EPSILON = 0.1
DESIRED_DISTANCE = 5.0
INTERACTION_RANGE = 1.2 * DESIRED_DISTANCE
BUMP_H = 0.2
POTENTIAL_GAIN = 5.0  # Symmetric action function: a = b = 5, c = 0.
POSITION_GAIN = 0.1
VELOCITY_GAIN = 2.0 * np.sqrt(POSITION_GAIN)
DT = 0.02
show_animation = True


def sigma_norm(vectors):
    """Smooth norm of vectors along the last axis (equation 8)."""
    squared_norm = np.sum(np.asarray(vectors) ** 2, axis=-1)
    # Rationalized form avoids cancellation close to zero.
    return squared_norm / (np.sqrt(1.0 + EPSILON * squared_norm) + 1.0)


def bump_function(z):
    """Smooth adjacency weight, with a finite cutoff at z = 1 (equation 10)."""
    phase = np.clip((np.asarray(z) - BUMP_H) / (1.0 - BUMP_H), 0.0, 1.0)
    return np.where(np.asarray(z) >= 0.0, 0.5 * (1.0 + np.cos(np.pi * phase)), 0.0)


def interaction_terms(positions, velocities):
    """Return separation/cohesion and alignment accelerations (equation 23).

    Both inputs have shape (number of agents, 2). All agents use the same
    state snapshot; self-interactions are excluded from the adjacency matrix.
    """
    displacement = positions[np.newaxis, :, :] - positions[:, np.newaxis, :]
    distance = sigma_norm(displacement)
    adjacency = bump_function(distance / sigma_norm([INTERACTION_RANGE]))
    np.fill_diagonal(adjacency, 0.0)

    offset = distance - sigma_norm([DESIRED_DISTANCE])
    action = POTENTIAL_GAIN * adjacency * offset / np.sqrt(1.0 + offset ** 2)
    direction = displacement / (1.0 + EPSILON * distance[:, :, np.newaxis])
    gradient = np.sum(action[:, :, np.newaxis] * direction, axis=1)

    velocity_difference = velocities[np.newaxis, :, :] - velocities[:, np.newaxis, :]
    alignment = np.sum(adjacency[:, :, np.newaxis] * velocity_difference, axis=1)
    return gradient, alignment


def flocking_control(positions, velocities, reference_position, reference_velocity):
    """Compute Algorithm 2 acceleration with linear navigation (equation 24)."""
    gradient, alignment = interaction_terms(positions, velocities)
    navigation = (-POSITION_GAIN * (positions - reference_position)
                  - VELOCITY_GAIN * (velocities - reference_velocity))
    return gradient + alignment + navigation


def simulate(simulation_time=40.0, dt=DT, seed=0):
    """Return times, position/velocity histories and moving reference positions.

    The fixed-seed perturbed grid avoids coincident initial agents. Controls
    are held over each step, integrating the double-integrator model exactly
    for that constant acceleration. The reference has constant velocity.
    """
    rng = np.random.default_rng(seed)
    x, y = np.meshgrid(np.arange(5), np.arange(5))
    initial_positions = 4.5 * np.column_stack((x.ravel(), y.ravel()))
    initial_positions += rng.uniform(-0.6, 0.6, initial_positions.shape)
    times = np.arange(int(round(simulation_time / dt)) + 1) * dt
    positions = np.empty((len(times), len(initial_positions), 2))
    velocities = np.empty_like(positions)
    positions[0] = initial_positions
    velocities[0] = rng.uniform(-1.0, 1.0, initial_positions.shape)
    reference_velocity = np.array([1.0, 0.5])
    reference_start = initial_positions.mean(axis=0) + [4.0, 2.0]
    references = reference_start + times[:, np.newaxis] * reference_velocity

    for k in range(len(times) - 1):
        acceleration = flocking_control(positions[k], velocities[k],
                                        references[k], reference_velocity)
        positions[k + 1] = positions[k] + dt * velocities[k] + 0.5 * dt ** 2 * acceleration
        velocities[k + 1] = velocities[k] + dt * acceleration
    return times, positions, velocities, references


def create_animation(times, positions, velocities, references):
    """Animate a simulation result; the returned animation can also be saved."""
    fig, (ax, error_ax) = plt.subplots(1, 2, figsize=(10, 4.5))
    colors = plt.colormaps["viridis"](np.linspace(0.1, 0.9, positions.shape[1]))
    edges = LineCollection([], colors="0.8", linewidths=0.7, zorder=1)
    ax.add_collection(edges)
    agents = ax.scatter(*positions[0].T, c=colors, s=25, zorder=3, label="Agents")
    arrows = ax.quiver(*positions[0].T, *velocities[0].T, color=colors,
                       angles="xy", scale_units="xy", scale=0.6)
    reference, = ax.plot([], [], "r*", markersize=12, label="Moving reference")
    ax.plot(*references.T, "r--", alpha=0.4)
    all_points = np.concatenate((positions.reshape(-1, 2), references))
    ax.set(xlim=(all_points[:, 0].min() - 3, all_points[:, 0].max() + 3),
           ylim=(all_points[:, 1].min() - 3, all_points[:, 1].max() + 3),
           xlabel="x [m]", ylabel="y [m]", aspect="equal")
    ax.legend(loc="upper left")
    ax.grid(True)

    reference_velocity = (references[-1] - references[0]) / (times[-1] - times[0])
    velocity_error = velocities - reference_velocity
    rms_error = np.sqrt(np.mean(np.sum(velocity_error ** 2, axis=2), axis=1))
    error_ax.plot(times, rms_error, color="0.85")
    error_line, = error_ax.plot([], [], color="tab:blue")
    error_ax.set(xlabel="Time [s]", ylabel="RMS velocity error [m/s]",
                 title="Velocity convergence", xlim=(times[0], times[-1]),
                 ylim=(0.0, max(0.1, rms_error.max() * 1.1)))
    error_ax.grid(True)
    fig.tight_layout()

    def update(k):
        q, p = positions[k], velocities[k]
        distance = np.linalg.norm(q[:, np.newaxis, :] - q[np.newaxis, :, :], axis=2)
        i, j = np.nonzero(np.triu(distance < INTERACTION_RANGE, k=1))
        edges.set_segments(np.stack((q[i], q[j]), axis=1))
        agents.set_offsets(q)
        arrows.set_offsets(q)
        arrows.set_UVC(*p.T)
        reference.set_data([references[k, 0]], [references[k, 1]])
        error_line.set_data(times[:k + 1], rms_error[:k + 1])
        ax.set_title(f"Olfati-Saber flocking: t = {times[k]:.1f} s")
        return edges, agents, arrows, reference, error_line

    stride = max(1, int(round(0.1 / (times[1] - times[0]))))
    frames = list(range(0, len(times) - 1, stride)) + [len(times) - 1]
    return FuncAnimation(fig, update, frames=frames, interval=100, repeat=False)


def main():
    """Run the flocking example with optional Matplotlib animation."""
    result = simulate()
    if show_animation:
        animation = create_animation(*result)
        plt.show()
        return animation
    return None


if __name__ == "__main__":
    main()
