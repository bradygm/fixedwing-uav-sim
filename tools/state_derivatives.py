import parameters.aerosonde_parameters as MAV
from angleConversions import Quaternion2Euler
import numpy as np
def _derivatives( state, delta, gamma, Va):

    # extract the states
    pn = state.item(0)
    pe = state.item(1)
    pd = state.item(2)
    u = state.item(3)
    v = state.item(4)
    w = state.item(5)
    e0 = state.item(6)
    e1 = state.item(7)
    e2 = state.item(8)
    e3 = state.item(9)
    phi, theta, psi = Quaternion2Euler(state[6:10])
    p = state.item(10)
    q = state.item(11)
    r = state.item(12)
    alpha = theta-gamma


    # Forces due to gravity
    # phi, theta, psi = Quaternion2Euler(self._state[6:10])
    # fx = -MAV.mass*MAV.gravity * np.sin(theta)
    # fy = MAV.mass*MAV.gravity*np.cos(theta)*np.sin(phi)
    # fz = MAV.mass*MAV.gravity*np.cos(theta)*np.cos(phi)
    fx = MAV.mass * MAV.gravity * 2 * (e1 * e3 - e2 * e0)
    fy = MAV.mass * MAV.gravity * 2 * (e2 * e3 + e1 * e0)
    fz = MAV.mass * MAV.gravity * (e3 ** 2 + e0 ** 2 - e1 ** 2 - e2 ** 2)

    # Forces due to aerodynamics
    cA = np.cos(alpha)
    sA = np.sin(alpha)
    Cd = MAV.C_D_p + (MAV.C_L_0 + MAV.C_L_alpha * alpha) ** 2 / (np.pi * np.exp(1) * MAV.AR)
    sigmaA = (1 + np.exp(-MAV.M * (alpha - MAV.alpha0)) + np.exp(MAV.M * (alpha + MAV.alpha0))) \
             / ((1 + np.exp(-MAV.M * (alpha - MAV.alpha0))) * (1 + np.exp(MAV.M * (alpha + MAV.alpha0))))
    Cl = (1 - sigmaA) * (MAV.C_L_0 + MAV.C_L_alpha * alpha) + sigmaA * (2 * np.sign(alpha) * sA ** 2 * cA)
    Cx = -Cd * cA + Cl * sA
    Cxq = -MAV.C_D_q * cA + MAV.C_L_q * sA
    Cxdeltae = -MAV.C_D_delta_e * cA + MAV.C_L_delta_e * sA
    Cz = -Cd * sA - Cl * cA
    Czq = -MAV.C_D_q * sA - MAV.C_L_q * cA
    Czdeltae = -MAV.C_D_delta_e * sA - MAV.C_L_delta_e * cA
    fx += .5 * MAV.rho * Va ** 2 * MAV.S_wing * (
                Cx + Cxq * MAV.c * q / (2 * Va) + Cxdeltae * delta[1])
    fy += .5 * MAV.rho * Va ** 2 * MAV.S_wing * (
                MAV.C_Y_0 + MAV.C_Y_beta * 0. + MAV.C_Y_p * MAV.b * p / (2 * Va) \
                + MAV.C_Y_r * MAV.b * r / (2 * Va) + MAV.C_Y_delta_a * delta[0] + MAV.C_Y_delta_r *
                delta[2])
    fz += .5 * MAV.rho * Va ** 2 * MAV.S_wing * (
                Cz + Czq * MAV.c * q / (2 * Va) + Czdeltae * delta[1])
    l = .5 * MAV.rho * Va ** 2 * MAV.S_wing * (MAV.b * (
                MAV.C_ell_0 + MAV.C_ell_beta * 0. + MAV.C_ell_p * MAV.b * p / (2 * Va) \
                + MAV.C_ell_r * MAV.b * r / (2 * Va) + MAV.C_ell_delta_a * delta[
                    0] + MAV.C_ell_delta_r * delta[2]))
    m = .5 * MAV.rho * Va ** 2 * MAV.S_wing * (MAV.c * (
                MAV.C_m_0 + MAV.C_m_alpha * alpha + MAV.C_m_q * MAV.c * q / (
                    2 * Va) + MAV.C_m_delta_e * delta[1]))
    n = .5 * MAV.rho * Va ** 2 * MAV.S_wing * (
                MAV.b * (MAV.C_n_0 + MAV.C_n_beta * 0. + MAV.C_n_p * MAV.b * p / (2 * Va) \
                         + MAV.C_n_r * MAV.b * r / (2 * Va) + MAV.C_n_delta_a * delta[
                             0] + MAV.C_n_delta_r * delta[2]))

    # Compute thrust and torque due to propeller
    # map delta_t throttle command (0 to 1) into motor input voltage
    V_in = MAV.V_max * delta[3]
    # Quadratic formula to solve for motor speed
    a = MAV.C_Q0 * MAV.rho * np.power(MAV.D_prop, 5) \
        / ((2. * np.pi) ** 2)
    b = (MAV.C_Q1 * MAV.rho * np.power(MAV.D_prop, 4) \
         / (2. * np.pi)) * Va + MAV.KQ ** 2 / MAV.R_motor
    c = MAV.C_Q2 * MAV.rho * np.power(MAV.D_prop, 3) \
        * Va ** 2 - (MAV.KQ / MAV.R_motor) * V_in + MAV.KQ * MAV.i0
    # Consider only positive _rotate_points
    Omega_op = (-b + np.sqrt(b ** 2. - 4. * a * c)) / (2. * a)
    # compute advance ratio
    J_op = 2. * np.pi * Va / (Omega_op * MAV.D_prop)
    # compute non-dimensionalized coefficients of thrust and torque
    C_T = MAV.C_T2 * J_op ** 2 + MAV.C_T1 * J_op + MAV.C_T0
    C_Q = MAV.C_Q2 * J_op ** 2 + MAV.C_Q1 * J_op + MAV.C_Q0
    # add thrust and torque due to propeller
    n = Omega_op / (2 * np.pi)
    fx += MAV.rho * n ** 2 * np.power(MAV.D_prop, 4) * C_T
    l += -MAV.rho * n ** 2 * np.power(MAV.D_prop, 5) * C_Q

    """
    for the dynamics xdot = f(x, u), returns f(x, u)
    """

    #   extract forces/moments
    # fx = forces_moments.item(0)
    # fy = forces_moments.item(1)
    # fz = forces_moments.item(2)
    # l = forces_moments.item(3)
    # m = forces_moments.item(4)
    # n = forces_moments.item(5)

    # position kinematics
    pn_dot = (e1 ** 2 + e0 ** 2 - e2 ** 2 - e3 ** 2) * u + \
             (2 * (e1 * e2 - e3 * e0)) * v + \
             (2 * (e1 * e3 + e2 * e0)) * w
    pe_dot = (2 * (e1 * e2 + e3 * e0)) * u + \
             (e2 ** 2 + e0 ** 2 - e1 ** 2 - e3 ** 2) * v + \
             (2 * (e2 * e3 - e1 * e0)) * w
    pd_dot = (2 * (e1 * e3 - e2 * e0)) * u + \
             (2 * (e2 * e3 + e1 * e0)) * v + \
             (e3 ** 2 + e0 ** 2 - e1 ** 2 - e2 ** 2) * w

    # position dynamics
    u_dot = r * v - q * w + fx / MAV.mass
    v_dot = p * w - r * u + fy / MAV.mass
    w_dot = q * u - p * v + fz / MAV.mass

    # rotational kinematics
    e0_dot = (-p * e1 - q * e2 - r * e3) / 2
    e1_dot = (p * e0 + r * e2 - q * e3) / 2
    e2_dot = (q * e0 - r * e1 + p * e3) / 2
    e3_dot = (r * e0 + q * e1 - p * e2) / 2

    # rotatonal dynamics
    p_dot = MAV.gamma1 * p * q - MAV.gamma2 * q * r + MAV.gamma3 * l + MAV.gamma4 * n
    q_dot = MAV.gamma5 * p * r - MAV.gamma6 * (p ** 2 - r ** 2) + m / MAV.Jy
    r_dot = MAV.gamma7 * p * q - MAV.gamma1 * q * r + MAV.gamma4 * l + MAV.gamma8 * n

    # collect the derivative of the states
    x_dot = np.array([[pn_dot, pe_dot, pd_dot, u_dot, v_dot, w_dot,
                       e0_dot, e1_dot, e2_dot, e3_dot, p_dot, q_dot, r_dot]]).T
    return x_dot