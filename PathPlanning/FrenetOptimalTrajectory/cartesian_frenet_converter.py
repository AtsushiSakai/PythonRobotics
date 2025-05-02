"""

A converter between Cartesian and Frenet coordinate systems

author: Wang Zheng (@Aglargil)

Reference:

- [Optimal Trajectory Generation for Dynamic Street Scenarios in a Frenet Frame]
(https://www.researchgate.net/profile/Moritz_Werling/publication/224156269_Optimal_Trajectory_Generation_for_Dynamic_Street_Scenarios_in_a_Frenet_Frame/links/54f749df0cf210398e9277af.pdf)

"""

import math


class CartesianFrenetConverter:
    """
    A class for converting states between Cartesian and Frenet coordinate systems
    """

    @ staticmethod
    def cartesian_to_frenet(rs, rx, ry, rtheta, rkappa, rdkappa, x, y, v, a, theta, kappa):
        """
        Convert state from Cartesian coordinate to Frenet coordinate

        Parameters
        ----------
            rs: reference line s-coordinate
            rx, ry: reference point coordinates
            rtheta: reference point heading
            rkappa: reference point curvature
            rdkappa: reference point curvature rate
            x, y: current position
            v: velocity
            a: acceleration
            theta: heading angle
            kappa: curvature

        Returns
        -------
            s_condition: [s(t), s'(t), s''(t)]
            d_condition: [d(s), d'(s), d''(s)]
        """
        dx = x - rx
        dy = y - ry

        cos_theta_r = math.cos(rtheta)
        sin_theta_r = math.sin(rtheta)

        cross_rd_nd = cos_theta_r * dy - sin_theta_r * dx
        d = math.copysign(math.hypot(dx, dy), cross_rd_nd)

        delta_theta = theta - rtheta
        tan_delta_theta = math.tan(delta_theta)
        cos_delta_theta = math.cos(delta_theta)

        one_minus_kappa_r_d = 1 - rkappa * d
        d_dot = one_minus_kappa_r_d * tan_delta_theta

        kappa_r_d_prime = rdkappa * d + rkappa * d_dot

        d_ddot = (-kappa_r_d_prime * tan_delta_theta +
                  one_minus_kappa_r_d / (cos_delta_theta * cos_delta_theta) *
                  (kappa * one_minus_kappa_r_d / cos_delta_theta - rkappa))

        s = rs
        s_dot = v * cos_delta_theta / one_minus_kappa_r_d

        delta_theta_prime = one_minus_kappa_r_d / cos_delta_theta * kappa - rkappa
        s_ddot = (a * cos_delta_theta -
                  s_dot * s_dot *
                  (d_dot * delta_theta_prime - kappa_r_d_prime)) / one_minus_kappa_r_d

        return [s, s_dot, s_ddot], [d, d_dot, d_ddot]

    @ staticmethod
    def frenet_to_cartesian(rs, rx, ry, rtheta, rkappa, rdkappa, s_condition, d_condition):
        """
        Convert state from Frenet coordinate to Cartesian coordinate

        Parameters
        ----------
            rs: reference line s-coordinate
            rx, ry: reference point coordinates
            rtheta: reference point heading
            rkappa: reference point curvature
            rdkappa: reference point curvature rate
            s_condition: [s(t), s'(t), s''(t)]
            d_condition: [d(s), d'(s), d''(s)]

        Returns
        -------
            x, y: position
            theta: heading angle
            kappa: curvature
            v: velocity
            a: acceleration
        """
        if abs(rs - s_condition[0]) >= 1.0e-6:
            raise ValueError(
                "The reference point s and s_condition[0] don't match")

        cos_theta_r = math.cos(rtheta)
        sin_theta_r = math.sin(rtheta)

        x = rx - sin_theta_r * d_condition[0]
        y = ry + cos_theta_r * d_condition[0]

        one_minus_kappa_r_d = 1 - rkappa * d_condition[0]

        tan_delta_theta = d_condition[1] / one_minus_kappa_r_d
        delta_theta = math.atan2(d_condition[1], one_minus_kappa_r_d)
        cos_delta_theta = math.cos(delta_theta)

        theta = CartesianFrenetConverter.normalize_angle(delta_theta + rtheta)

        kappa_r_d_prime = rdkappa * d_condition[0] + rkappa * d_condition[1]

        kappa = (((d_condition[2] + kappa_r_d_prime * tan_delta_theta) *
                  cos_delta_theta * cos_delta_theta) / one_minus_kappa_r_d + rkappa) * \
            cos_delta_theta / one_minus_kappa_r_d

        d_dot = d_condition[1] * s_condition[1]
        v = math.sqrt(one_minus_kappa_r_d * one_minus_kappa_r_d *
                      s_condition[1] * s_condition[1] + d_dot * d_dot)

        delta_theta_prime = one_minus_kappa_r_d / cos_delta_theta * kappa - rkappa

        a = (s_condition[2] * one_minus_kappa_r_d / cos_delta_theta +
             s_condition[1] * s_condition[1] / cos_delta_theta *
             (d_condition[1] * delta_theta_prime - kappa_r_d_prime))

        return x, y, theta, kappa, v, a

    @ staticmethod
    def normalize_angle(angle):
        """
        Normalize angle to [-pi, pi]
        """
        a = math.fmod(angle + math.pi, 2.0 * math.pi)
        if a < 0.0:
            a += 2.0 * math.pi
        return a - math.pi
