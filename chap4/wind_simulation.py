"""
Class to determine wind velocity at any given moment,
calculates a steady wind speed and uses a stochastic
process to represent wind gusts. (Follows section 4.4 in uav book)
"""
import sys
sys.path.append('..')
import numpy as np
import parameters.simulation_parameters as SIM

class wind_simulation:
    def __init__(self, Ts):
        # steady state wind defined in the inertial frame
        self._steady_state = np.array([[0., 0., 0.]]).T
        # self.steady_state = np.array([[3., 1., 0.]]).T

        #   Dryden gust model parameters (pg 56 UAV book)
        # HACK:  Setting Va to a constant value is a hack.  We set a nominal airspeed for the gust model.
        # Could pass current Va into the gust function and recalculate A and B matrices.
        Va = 17
        coeff_u = SIM.sigma_u*np.sqrt(2*Va/SIM.Lu)
        coeff_v = SIM.sigma_v*np.sqrt(2*Va/SIM.Lv)
        coeff_w = SIM.sigma_w*np.sqrt(2*Va/SIM.Lw)
        ua = coeff_u
        ub = Va/SIM.Lu
        va = coeff_v
        vb = coeff_v * Va/(SIM.Lv*np.sqrt(3))
        vc = 2*Va/SIM.Lv
        vd = (Va/SIM.Lv)**2
        wa = coeff_w
        wb = coeff_w * Va/(SIM.Lw*np.sqrt(3))
        wc = 2*Va/SIM.Lw
        wd = (Va/SIM.Lw)**2


        self._A = np.array([[(-ub), 0., 0., 0., 0.], \
                            [0., (-vc), (-vd), 0., 0.], \
                            [0., 1., 0., 0., 0.], \
                            [0., 0., 0., (-wc), (-wd)], \
                            [0., 0., 0., 1., 0.]])
        self._B = np.array([[1.],[1.],[0.],[1.],[0.]])
        self._C = np.array([[ua, 0., 0., 0., 0.],[0., va, vb, 0., 0.],[0., 0., 0., wa, wb]])
        self._gust_state = np.array([[0., 0., 0., 0., 0.]]).T
        self._Ts = Ts

    def update(self):
        # returns a six vector.
        #   The first three elements are the steady state wind in the inertial frame
        #   The second three elements are the gust in the body frame
        # return np.array([[0., 0., 0., 0., 0., 0.]]).T
        return np.concatenate(( self._steady_state, self._gust() ))

    def _gust(self):
        # calculate wind gust using Dryden model.  Gust is defined in the body frame
        w = np.random.randn()  # zero mean unit variance Gaussian (white noise)
        w1 = np.random.randn()  # zero mean unit variance Gaussian (white noise)
        w2 = np.random.randn()  # zero mean unit variance Gaussian (white noise)
        # propagate Dryden model (Euler method): x[k+1] = x[k] + Ts*( A x[k] + B w[k] )
        self._gust_state += self._Ts * (self._A @ self._gust_state + self._B * np.array([[w, w1, 0., w2, 0.]]).T)
        # output the current gust: y[k] = C x[k]
        return self._C @ self._gust_state
