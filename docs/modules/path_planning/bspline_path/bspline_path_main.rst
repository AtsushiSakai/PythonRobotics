B-Spline planning
-----------------

.. image:: Figure_1.png

This is a path planning with B-Spline curse.

If you input waypoints, it generates a smooth path with B-Spline curve.

The final course should be on the first and last waypoints.

Bspline basics
~~~~~~~~~~~~~~

.. image:: basis_functions.png

.. code-block:: python

	from scipy.interpolate import BSpline

	def B_orig(x, k, i, t):
		if k == 0:
			return 1.0 if t[i] <= x < t[i + 1] else 0.0
		if t[i + k] == t[i]:
			c1 = 0.0
		else:
			c1 = (x - t[i]) / (t[i + k] - t[i]) * B(x, k - 1, i, t)

		if t[i + k + 1] == t[i + 1]:
			c2 = 0.0
		else:
			c2 = (t[i + k + 1] - x) / (t[i + k + 1] - t[i + 1]) * B(x, k - 1, i + 1, t)
		return c1 + c2


	def B(x, k, i, t):
		c = np.zeros_like(t)
		c[i] = 1
		return BSpline(t, c, k)(x)


	def main():
		k = 3  # degree of the spline
		t = [0, 1, 2, 3, 4, 5]  # knots vector

		x = np.linspace(0, 5, 1000, endpoint=False)
		t = np.r_[[np.min(t)]*k, t, [np.max(t)]*k]

		n = len(t) - k - 1
		for i in range(n):
			y = np.array([B(ix, k, i, t) for ix in x])
			plt.plot(x, y, label=f'i = {i}')

		plt.title(f'Basis functions (k = {k}, knots = {t})')
		plt.show()

Bspline interpolation planning
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. image:: interpolation1.png

Bspline approximation planning
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. image:: approximation1.png


References
~~~~~~~~~~

-  `B-spline - Wikipedia <https://en.wikipedia.org/wiki/B-spline>`__
