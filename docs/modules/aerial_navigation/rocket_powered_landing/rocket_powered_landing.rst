Rocket powered landing
-----------------------------

Simulation
~~~~~~~~~~

.. code-block:: ipython3

    from IPython.display import Image
    Image(filename="figure.png",width=600)
    from IPython.display import display, HTML
    
    display(HTML(data="""
    <style>
        div#notebook-container    { width: 95%; }
        div#menubar-container     { width: 65%; }
        div#maintoolbar-container { width: 99%; }
    </style>
    """))



.. raw:: html

    
    <style>
        div#notebook-container    { width: 95%; }
        div#menubar-container     { width: 65%; }
        div#maintoolbar-container { width: 99%; }
    </style>



.. figure:: https://github.com/AtsushiSakai/PythonRoboticsGifs/raw/master/AerialNavigation/rocket_powered_landing/animation.gif
   :alt: gif

Equation generation
~~~~~~~~~~~~~~~~~~~

.. code-block:: ipython3

    import sympy as sp
    import numpy as np
    from IPython.display import display
    sp.init_printing(use_latex='mathjax')

.. code-block:: ipython3

    # parameters
    # Angular moment of inertia
    J_B = 1e-2 * np.diag([1., 1., 1.])
    
    # Gravity
    g_I = np.array((-1, 0., 0.))
    
    # Fuel consumption
    alpha_m = 0.01
    
    # Vector from thrust point to CoM
    r_T_B = np.array([-1e-2, 0., 0.])
    
    
    def dir_cosine(q):
            return np.matrix([
                [1 - 2 * (q[2] ** 2 + q[3] ** 2), 2 * (q[1] * q[2] +
                                                       q[0] * q[3]), 2 * (q[1] * q[3] - q[0] * q[2])],
                [2 * (q[1] * q[2] - q[0] * q[3]), 1 - 2 *
                 (q[1] ** 2 + q[3] ** 2), 2 * (q[2] * q[3] + q[0] * q[1])],
                [2 * (q[1] * q[3] + q[0] * q[2]), 2 * (q[2] * q[3] -
                                                       q[0] * q[1]), 1 - 2 * (q[1] ** 2 + q[2] ** 2)]
            ])
    
    def omega(w):
            return np.matrix([
                [0, -w[0], -w[1], -w[2]],
                [w[0], 0, w[2], -w[1]],
                [w[1], -w[2], 0, w[0]],
                [w[2], w[1], -w[0], 0],
            ])
    
    def skew(v):
        return np.matrix([
                [0, -v[2], v[1]],
                [v[2], 0, -v[0]],
                [-v[1], v[0], 0]
            ])
    


.. code-block:: ipython3

    f = sp.zeros(14, 1)
    
    x = sp.Matrix(sp.symbols(
        'm rx ry rz vx vy vz q0 q1 q2 q3 wx wy wz', real=True))
    u = sp.Matrix(sp.symbols('ux uy uz', real=True))
    
    g_I = sp.Matrix(g_I)
    r_T_B = sp.Matrix(r_T_B)
    J_B = sp.Matrix(J_B)
    
    C_B_I = dir_cosine(x[7:11, 0])
    C_I_B = C_B_I.transpose()
    
    f[0, 0] = - alpha_m * u.norm()
    f[1:4, 0] = x[4:7, 0]
    f[4:7, 0] = 1 / x[0, 0] * C_I_B * u + g_I
    f[7:11, 0] = 1 / 2 * omega(x[11:14, 0]) * x[7: 11, 0]
    f[11:14, 0] = J_B ** -1 * \
        (skew(r_T_B) * u - skew(x[11:14, 0]) * J_B * x[11:14, 0])


.. code-block:: ipython3

    display(sp.simplify(f)) # f



.. math::

    \left[\begin{matrix}- 0.01 \sqrt{ux^{2} + uy^{2} + uz^{2}}\\vx\\vy\\vz\\\frac{- 1.0 m - ux \left(2 q_{2}^{2} + 2 q_{3}^{2} - 1\right) - 2 uy \left(q_{0} q_{3} - q_{1} q_{2}\right) + 2 uz \left(q_{0} q_{2} + q_{1} q_{3}\right)}{m}\\\frac{2 ux \left(q_{0} q_{3} + q_{1} q_{2}\right) - uy \left(2 q_{1}^{2} + 2 q_{3}^{2} - 1\right) - 2 uz \left(q_{0} q_{1} - q_{2} q_{3}\right)}{m}\\\frac{- 2 ux \left(q_{0} q_{2} - q_{1} q_{3}\right) + 2 uy \left(q_{0} q_{1} + q_{2} q_{3}\right) - uz \left(2 q_{1}^{2} + 2 q_{2}^{2} - 1\right)}{m}\\- 0.5 q_{1} wx - 0.5 q_{2} wy - 0.5 q_{3} wz\\0.5 q_{0} wx + 0.5 q_{2} wz - 0.5 q_{3} wy\\0.5 q_{0} wy - 0.5 q_{1} wz + 0.5 q_{3} wx\\0.5 q_{0} wz + 0.5 q_{1} wy - 0.5 q_{2} wx\\0\\1.0 uz\\- 1.0 uy\end{matrix}\right]


.. code-block:: ipython3

    display(sp.simplify(f.jacobian(x)))# A 



.. math::

    \left[\begin{array}{cccccccccccccc}0 & 0 & 0 & 0 & 0 & 0 & 0 & 0 & 0 & 0 & 0 & 0 & 0 & 0\\0 & 0 & 0 & 0 & 1 & 0 & 0 & 0 & 0 & 0 & 0 & 0 & 0 & 0\\0 & 0 & 0 & 0 & 0 & 1 & 0 & 0 & 0 & 0 & 0 & 0 & 0 & 0\\0 & 0 & 0 & 0 & 0 & 0 & 1 & 0 & 0 & 0 & 0 & 0 & 0 & 0\\\frac{ux \left(2 q_{2}^{2} + 2 q_{3}^{2} - 1\right) + 2 uy \left(q_{0} q_{3} - q_{1} q_{2}\right) - 2 uz \left(q_{0} q_{2} + q_{1} q_{3}\right)}{m^{2}} & 0 & 0 & 0 & 0 & 0 & 0 & \frac{2 \left(q_{2} uz - q_{3} uy\right)}{m} & \frac{2 \left(q_{2} uy + q_{3} uz\right)}{m} & \frac{2 \left(q_{0} uz + q_{1} uy - 2 q_{2} ux\right)}{m} & \frac{2 \left(- q_{0} uy + q_{1} uz - 2 q_{3} ux\right)}{m} & 0 & 0 & 0\\\frac{- 2 ux \left(q_{0} q_{3} + q_{1} q_{2}\right) + uy \left(2 q_{1}^{2} + 2 q_{3}^{2} - 1\right) + 2 uz \left(q_{0} q_{1} - q_{2} q_{3}\right)}{m^{2}} & 0 & 0 & 0 & 0 & 0 & 0 & \frac{2 \left(- q_{1} uz + q_{3} ux\right)}{m} & \frac{2 \left(- q_{0} uz - 2 q_{1} uy + q_{2} ux\right)}{m} & \frac{2 \left(q_{1} ux + q_{3} uz\right)}{m} & \frac{2 \left(q_{0} ux + q_{2} uz - 2 q_{3} uy\right)}{m} & 0 & 0 & 0\\\frac{2 ux \left(q_{0} q_{2} - q_{1} q_{3}\right) - 2 uy \left(q_{0} q_{1} + q_{2} q_{3}\right) + uz \left(2 q_{1}^{2} + 2 q_{2}^{2} - 1\right)}{m^{2}} & 0 & 0 & 0 & 0 & 0 & 0 & \frac{2 \left(q_{1} uy - q_{2} ux\right)}{m} & \frac{2 \left(q_{0} uy - 2 q_{1} uz + q_{3} ux\right)}{m} & \frac{2 \left(- q_{0} ux - 2 q_{2} uz + q_{3} uy\right)}{m} & \frac{2 \left(q_{1} ux + q_{2} uy\right)}{m} & 0 & 0 & 0\\0 & 0 & 0 & 0 & 0 & 0 & 0 & 0 & - 0.5 wx & - 0.5 wy & - 0.5 wz & - 0.5 q_{1} & - 0.5 q_{2} & - 0.5 q_{3}\\0 & 0 & 0 & 0 & 0 & 0 & 0 & 0.5 wx & 0 & 0.5 wz & - 0.5 wy & 0.5 q_{0} & - 0.5 q_{3} & 0.5 q_{2}\\0 & 0 & 0 & 0 & 0 & 0 & 0 & 0.5 wy & - 0.5 wz & 0 & 0.5 wx & 0.5 q_{3} & 0.5 q_{0} & - 0.5 q_{1}\\0 & 0 & 0 & 0 & 0 & 0 & 0 & 0.5 wz & 0.5 wy & - 0.5 wx & 0 & - 0.5 q_{2} & 0.5 q_{1} & 0.5 q_{0}\\0 & 0 & 0 & 0 & 0 & 0 & 0 & 0 & 0 & 0 & 0 & 0 & 0 & 0\\0 & 0 & 0 & 0 & 0 & 0 & 0 & 0 & 0 & 0 & 0 & 0 & 0 & 0\\0 & 0 & 0 & 0 & 0 & 0 & 0 & 0 & 0 & 0 & 0 & 0 & 0 & 0\end{array}\right]


.. code-block:: ipython3

    sp.simplify(f.jacobian(u)) # B




.. math::

    \left[\begin{matrix}- \frac{0.01 ux}{\sqrt{ux^{2} + uy^{2} + uz^{2}}} & - \frac{0.01 uy}{\sqrt{ux^{2} + uy^{2} + uz^{2}}} & - \frac{0.01 uz}{\sqrt{ux^{2} + uy^{2} + uz^{2}}}\\0 & 0 & 0\\0 & 0 & 0\\0 & 0 & 0\\\frac{- 2 q_{2}^{2} - 2 q_{3}^{2} + 1}{m} & \frac{2 \left(- q_{0} q_{3} + q_{1} q_{2}\right)}{m} & \frac{2 \left(q_{0} q_{2} + q_{1} q_{3}\right)}{m}\\\frac{2 \left(q_{0} q_{3} + q_{1} q_{2}\right)}{m} & \frac{- 2 q_{1}^{2} - 2 q_{3}^{2} + 1}{m} & \frac{2 \left(- q_{0} q_{1} + q_{2} q_{3}\right)}{m}\\\frac{2 \left(- q_{0} q_{2} + q_{1} q_{3}\right)}{m} & \frac{2 \left(q_{0} q_{1} + q_{2} q_{3}\right)}{m} & \frac{- 2 q_{1}^{2} - 2 q_{2}^{2} + 1}{m}\\0 & 0 & 0\\0 & 0 & 0\\0 & 0 & 0\\0 & 0 & 0\\0 & 0 & 0\\0 & 0 & 1.0\\0 & -1.0 & 0\end{matrix}\right]



References
~~~~~~~~~~

-  Python implementation of ‘Successive Convexification for 6-DoF Mars
   Rocket Powered Landing with Free-Final-Time’ paper by Michael Szmuk
   and Behçet Açıkmeşe.

-  inspired by EmbersArc/SuccessiveConvexificationFreeFinalTime:
   Implementation of “Successive Convexification for 6-DoF Mars Rocket
   Powered Landing with Free-Final-Time”
   https://github.com/EmbersArc/SuccessiveConvexificationFreeFinalTime
