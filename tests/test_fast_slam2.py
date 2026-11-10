import conftest
import numpy as np
from SLAM.FastSLAM2 import fast_slam2 as m


def test1():
    m.show_animation = False
    m.SIM_TIME = 3.0
    m.main()


def test_resampling_preserves_parent_states():
    particles = [m.Particle(1) for _ in range(m.N_PARTICLE)]
    for i, particle in enumerate(particles):
        particle.x, particle.y, particle.yaw = float(i), -float(i), i * 0.01
        particle.lm[:] = i
        particle.lmP[:] = i + 10
        particle.w = 0.4 if i == 0 else (0.6 if i == 1 else 0.0)

    result = m.resampling(particles)
    expected_parents = np.repeat([0, 1], [40, 60])
    np.testing.assert_array_equal([p.x for p in result], expected_parents)
    np.testing.assert_array_equal([p.y for p in result], -expected_parents)
    np.testing.assert_allclose([p.yaw for p in result], expected_parents * 0.01)
    for particle, parent in zip(result, expected_parents):
        np.testing.assert_array_equal(particle.lm, np.full((1, 2), parent))
        np.testing.assert_array_equal(particle.lmP, np.full((2, 2), parent + 10))
        assert particle.w == 1.0 / m.N_PARTICLE


def test_resampled_landmark_updates_do_not_change_siblings():
    particles = [m.Particle(1) for _ in range(m.N_PARTICLE)]
    for particle in particles:
        particle.w = 0.0
    particles[-1].w = 1.0
    particles[-1].lm[:] = [1.0, 2.0]
    particles[-1].lmP[:] = np.eye(2)

    result = m.resampling(particles)
    sibling_landmarks = result[1].lm.copy()
    sibling_covariance = result[1].lmP.copy()
    m.add_new_lm(result[0], np.array([5.0, 0.0, 0.0]), m.Q)
    np.testing.assert_array_equal(result[1].lm, sibling_landmarks)
    np.testing.assert_array_equal(result[1].lmP, sibling_covariance)


def test_uniform_weights_keep_particle_states():
    particles = [m.Particle(1) for _ in range(m.N_PARTICLE)]
    for i, particle in enumerate(particles):
        particle.x = float(i)
        particle.lm[:] = i
    original_particles = particles.copy()

    result = m.resampling(particles)
    assert result is particles
    assert all(p is original for p, original in zip(result, original_particles))
    np.testing.assert_array_equal([p.x for p in result], np.arange(m.N_PARTICLE))
    for i, particle in enumerate(result):
        np.testing.assert_array_equal(particle.lm, np.full((1, 2), i))


if __name__ == '__main__':
    conftest.run_this_test(__file__)
