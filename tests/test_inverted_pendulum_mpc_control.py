
import sys
if 'cvxpy' in sys.modules:  # pragma: no cover

    from InvertedPendulumCart.InvertedPendulumMPCControl \
        import inverted_pendulum_mpc_control as m

    def test1():
        m.show_animation = False
        m.main()
