"""
mav_dynamics
    - this file implements the dynamic equations of motion for MAV
    - use unit quaternion for the attitude state

"""
import sys
sys.path.append('..')
import numpy as np

# load message types
from message_types.msg_state import msg_state

import parameters.aerosonde_parameters as MAV
from angleConversions import Quaternion2Euler, Quaternion2Rotation

class mav_dynamics:
    def __init__(self, Ts):
        self._ts_simulation = Ts
        # set initial states based on parameter file
        # _state is the 13x1 internal state of the aircraft that is being propagated:
        # _state = [pn, pe, pd, u, v, w, e0, e1, e2, e3, p, q, r]
        # We will also need a variety of other elements that are functions of the _state and the wind.
        # self.true_state is a 19x1 vector that is estimated and used by the autopilot to control the aircraft:
        # true_state = [pn, pe, h, Va, alpha, beta, phi, theta, chi, p, q, r, Vg, wn, we, psi, gyro_bx, gyro_by, gyro_bz]
        self._state = np.array([[MAV.pn0],  # (0)
                               [MAV.pe0],   # (1)
                               [MAV.pd0],   # (2)
                               [MAV.u0],    # (3)
                               [MAV.v0],    # (4)
                               [MAV.w0],    # (5)
                               [MAV.e0],    # (6)
                               [MAV.e1],    # (7)
                               [MAV.e2],    # (8)
                               [MAV.e3],    # (9)
                               [MAV.p0],    # (10)
                               [MAV.q0],    # (11)
                               [MAV.r0]])   # (12)
        # store wind data for fast recall since it is used at various points in simulation
        self._wind = np.array([[0.], [0.], [0.]])  # wind in NED frame in meters/sec
        self._update_velocity_data()
        # store forces to avoid recalculation in the sensors function
        self._forces = np.array([[0.], [0.], [0.]])
        self._Va = MAV.u0
        self._alpha = 0
        self._beta = 0
        # initialize true_state message
        self.msg_true_state = msg_state()

    ###################################
    # public functions
    def update_state(self, delta, wind):
        '''
            Integrate the differential equations defining dynamics, update sensors
            delta = (delta_a, delta_e, delta_r, delta_t) are the control inputs
            wind is the wind vector in inertial coordinates
            Ts is the time step between function calls.
        '''
        # get forces and moments acting on rigid bod
        forces_moments = self._forces_moments(delta)

        # Integrate ODE using Runge-Kutta RK4 algorithm
        time_step = self._ts_simulation
        k1 = self._derivatives(self._state, forces_moments)
        k2 = self._derivatives(self._state + time_step/2.*k1, forces_moments)
        k3 = self._derivatives(self._state + time_step/2.*k2, forces_moments)
        k4 = self._derivatives(self._state + time_step*k3, forces_moments)
        self._state += time_step/6 * (k1 + 2*k2 + 2*k3 + k4)

        # normalize the quaternion
        e0 = self._state.item(6)
        e1 = self._state.item(7)
        e2 = self._state.item(8)
        e3 = self._state.item(9)
        normE = np.sqrt(e0**2+e1**2+e2**2+e3**2)
        self._state[6][0] = self._state.item(6)/normE
        self._state[7][0] = self._state.item(7)/normE
        self._state[8][0] = self._state.item(8)/normE
        self._state[9][0] = self._state.item(9)/normE

        # update the airspeed, angle of attack, and side slip angles using new state
        self._update_velocity_data(wind)

        # update the message class for the true state
        self._update_msg_true_state()

    ###################################
    # private functions
    def _derivatives(self, state, forces_moments):
        """
        for the dynamics xdot = f(x, u), returns f(x, u)
        """
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
        p = state.item(10)
        q = state.item(11)
        r = state.item(12)
        #   extract forces/moments
        fx = forces_moments.item(0)
        fy = forces_moments.item(1)
        fz = forces_moments.item(2)
        l = forces_moments.item(3)
        m = forces_moments.item(4)
        n = forces_moments.item(5)

        # position kinematics
        pn_dot = (e1**2 + e0**2 - e2**2 - e3**2)*u + \
                 (2*(e1*e2-e3*e0))*v + \
                 (2*(e1*e3 + e2*e0))*w
        pe_dot = (2*(e1*e2 + e3*e0))*u + \
                 (e2**2 + e0**2 - e1**2 - e3**2)*v + \
                 (2*(e2*e3 - e1*e0))*w
        pd_dot = (2*(e1*e3 - e2*e0))*u + \
                 (2*(e2*e3 + e1*e0))*v + \
                 (e3**2 + e0**2 - e1**2 - e2**2)*w

        # position dynamics
        u_dot = r*v - q*w + fx/MAV.mass
        v_dot = p*w - r*u + fy/MAV.mass
        w_dot = q*u - p*v + fz/MAV.mass

        # rotational kinematics
        e0_dot = (-p*e1 - q*e2 - r*e3)/2
        e1_dot = (p*e0 + r*e2 - q*e3)/2
        e2_dot = (q*e0 - r*e1 + p*e3)/2
        e3_dot = (r*e0 + q*e1 - p*e2)/2

        # rotatonal dynamics
        p_dot = MAV.gamma1*p*q - MAV.gamma2*q*r + MAV.gamma3*l + MAV.gamma4*n
        q_dot = MAV.gamma5*p*r - MAV.gamma6*(p**2-r**2) + m/MAV.Jy
        r_dot = MAV.gamma7*p*q - MAV.gamma1*q*r + MAV.gamma4*l + MAV.gamma8*n

        # collect the derivative of the states
        x_dot = np.array([[pn_dot, pe_dot, pd_dot, u_dot, v_dot, w_dot,
                           e0_dot, e1_dot, e2_dot, e3_dot, p_dot, q_dot, r_dot]]).T
        return x_dot

    def _update_velocity_data(self, wind=np.zeros((6,1))):
        # compute airspeed
        self._Va =
        # compute angle of attack
        self._alpha =
        # compute sideslip angle
        self._beta =

    def _forces_moments(self, delta):
        """
        return the forces on the UAV based on the state, wind, and control surfaces
        :param delta: np.matrix(delta_a, delta_e, delta_r, delta_t)
        :return: Forces and Moments on the UAV np.matrix(Fx, Fy, Fz, Ml, Mn, Mm)
        """
        #Forces due to gravity
        phi, theta, psi = Quaternion2Euler(self._state[6:10])
        fx = -MAV.mass*MAV.gravity * sin(theta)
        fy = MAV.mass*MAV.gravity*cos(theta)*sin(phi)
        fz = MAV.mass*MAV.gravity*cos(theta)*cos(phi)

        #Forces due to aerodynamics
        cA = cos(self._alpha)
        sA = sin(self._alpha)
        fx +=
        fy +=
        fz +=



        #Compute thrust and torque due to propeller
        #map delta_t throttle command (0 to 1) into motor input voltage
        V_in = MAV.V_max*delta[1]
        # Quadratic formula to solve for motor speed
        a = MAV.C_Q0 * MAV.rho * np.power(MAV.D_prop, 5) \
            / ((2.*np.pi)**2)
        b = (MAV.C_Q1 * MAV.rho * np.power(MAV.D_prop,4) \
            / (2.*np.pi)) * self._Va + KQ**2/MAV.R_motor
        c = MAV.C_Q2 * MAV.rho * np.power(MAV.D_prop,3) \
            * self._Va**2 - (KQ/MAV.R_motor) * Volts + KQ*MAV.i0
        #Consider only positive _rotate_points
        Omega_op = (-b + np.sqrt(b**2 - 4*a*c))/ (2.*a)
        #compute advance ratio
        J_op = 2*np.pi*self._Va / (Omega_op * MAV.D_prop)
        #compute non-dimensionalized coefficients of thrust and torque
        C_T = MAV.C_T2*J_op**2 + MAV.C_T1*J_op + MAV.C_T0
        C_Q = MAV.C_Q2*J_op**2 + MAV.C_Q1*J_op + MAV.C_Q0
        #add thrust and torque due to propeller
        n = Omega_op / (2*np.pi)
        fx += MAV.rho * n**2 * np.power(MAV.D_prop,4) * C.T
        Mx += -MAV.rho * n**2 * np.power(MAV.D_prop,5) * C.Q

        self._forces[0] = self.fx
        self._forces[1] = fy
        self._forces[2] = fz
        return np.array([[fx, fy, fz, Mx, My, Mz]]).T

    def _update_msg_true_state(self):
        # update the class structure for the true state:
        #   [pn, pe, h, Va, alpha, beta, phi, theta, chi, p, q, r, Vg, wn, we, psi, gyro_bx, gyro_by, gyro_bz]
        phi, theta, psi = Quaternion2Euler(self._state[6:10])
        self.msg_true_state.pn = self._state.item(0)
        self.msg_true_state.pe = self._state.item(1)
        self.msg_true_state.h = -self._state.item(2)
        self.msg_true_state.Va = self._Va
        self.msg_true_state.alpha = self._alpha
        self.msg_true_state.beta = self._beta
        self.msg_true_state.phi = phi
        self.msg_true_state.theta = theta
        self.msg_true_state.psi = psi
        self.msg_true_state.Vg =
        self.msg_true_state.gamma =
        self.msg_true_state.chi =
        self.msg_true_state.p = self._state.item(10)
        self.msg_true_state.q = self._state.item(11)
        self.msg_true_state.r = self._state.item(12)
        self.msg_true_state.wn = self._wind.item(0)
        self.msg_true_state.we = self._wind.item(1)
