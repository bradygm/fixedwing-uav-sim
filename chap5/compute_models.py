"""
compute_ss_model
    - Chapter 5 assignment for Beard & McLain, PUP, 2012
    - Update history:  
        2/4/2019 - RWB
"""
import sys
sys.path.append('..')
import numpy as np
from scipy.optimize import minimize
import parameters.aerosonde_parameters as MAV
from parameters.simulation_parameters import ts_simulation as Ts
sys.path.append('../tools')
from angleConversions import Euler2Quaternion, Quaternion2Euler


def compute_tf_model(mav, trim_state, trim_input):
    # trim values
    T_phi_delta_a = a_phi_2/()

    return T_phi_delta_a, T_chi_phi, T_theta_delta_e, T_h_theta, T_h_Va, T_Va_delta_t, T_Va_theta, T_beta_delta_r

def compute_ss_model(mav, trim_state, trim_input):
    A_q = df_dx(mav, trim_state, trim_input)
    B_q = df_du(mav, trim_state, trim_input)
    dT_dxq = dT_d_x_q(trim_state[6:10])
    phi, theta, psi = Quaternion2Euler(trim_state[6:10])
    dT_inv_dxq = dT_inv_d_x_q(phi, theta, psi)
    A_e = dT_dxq @ A_q @ dT_inv_dxq
    B_e = dT_dxq @ B_q

    A_lat = A_e[[[4],[9],[11],[6],[8]],[4,9,11,6,8]]
    B_lat = B_e[[[4],[9],[11],[6],[8]],[0,2]]
    A_lon = A_e[[[3],[5],[10],[7],[2]],[3,5,10,7,2]] #WHAT ABOUT HDOT!! negative?
    B_lon = B_e[[[3],[5],[10],[7],[2]],[1,2]]

    return A_lon, B_lon, A_lat, B_lat #B_lat only one completely right

def euler_state(x_quat):
    # convert state x with attitude represented by quaternion
    # to x_euler with attitude represented by Euler angles
    phi, theta, psi = Quaternion2Euler(x_quat[6:10])
    x_euler = np.array([[x_quat[0]],
                        [x_quat[1]],
                        [x_quat[2]],
                        [x_quat[3]],
                        [x_quat[4]],
                        [x_quat[5]],
                        [phi],
                        [theta],
                        [psi],
                        [x_quat[10]],
                        [x_quat[11]],
                        [x_quat[12]]])
    return x_euler

def quaternion_state(x_euler):
    # convert state x_euler with attitude represented by Euler angles
    # to x_quat with attitude represented by quaternions
    e0, e1, e2, e3 = Euler2Quaternion(x_euler[6], x_euler[7], x_euler[8])
    x_quat = np.array([[x_euler[0]],
                        [x_euler[1]],
                        [x_euler[2]],
                        [x_euler[3]],
                        [x_euler[4]],
                        [x_euler[5]],
                        [e0],
                        [e1],
                        [e2],
                        [e3],
                        [x_euler[9]],
                        [x_euler[10]],
                        [x_euler[11]]])
    return x_quat

def f_euler(mav, x_euler, input):
    # return 12x1 dynamics (as if state were Euler state)
    # compute f at euler_state
    return f_euler_

def df_dx(mav, x_quat, delta):
    # take partial of f_quat with respect to x_quat
    eps = .01  # deviation
    A = np.zeros((13, 13))  # Jacobian of f wrt x
    mav._state = x_quat
    mav._update_velocity_data()
    f_at_x = mav._derivatives(x_quat, mav._forces_moments(delta))
    for i in range(0, 13):
        x_eps = np.copy(x_quat)
        x_eps[i][0] += eps  # add eps to the ith state
        mav._state = x_eps
        mav._update_velocity_data()
        f_at_x_eps = mav._derivatives(x_eps, mav._forces_moments(delta))
        df_dxi = (f_at_x_eps - f_at_x) / eps
        A[:, i] = df_dxi[:, 0]
    return A

def df_du(mav, x_quat, delta):
    # take partial of f_quat with respect to delta
    eps = .01  # deviation
    B = np.zeros((13, 4))  # Jacobian of f wrt x
    mav._state = x_quat
    mav._update_velocity_data()
    f_at_x = mav._derivatives(x_quat, mav._forces_moments(delta))
    for i in range(0, 4):
        u_eps = np.copy(delta)
        u_eps[i][0] += eps  # add eps to the ith state
        mav._state = x_quat
        mav._update_velocity_data()
        f_at_x_eps = mav._derivatives(x_quat, mav._forces_moments(u_eps))
        df_dxi = (f_at_x_eps - f_at_x) / eps
        B[:, i] = df_dxi[:, 0]
    return B

def dT_d_x_q(e):
    eps = .01
    dtheta_de = np.zeros((3, 4))
    phi_at_e, theta_at_e, psi_at_e = Quaternion2Euler(e)
    for i in range(0, 4):
        e_eps = np.copy(e)
        e_eps[i][0] += eps  # add eps to the ith state
        phi_at_e_eps, theta_at_e_eps, psi_at_e_eps = Quaternion2Euler(e_eps)
        d_theta_dxi = np.array([[(phi_at_e_eps - phi_at_e) / eps],
                                [(theta_at_e_eps - theta_at_e) / eps],
                                [(psi_at_e_eps - psi_at_e_eps) / eps]])
        dtheta_de[:, i] = d_theta_dxi[:, 0]
    eye3_3 = np.eye(3)
    zero3_3 = np.zeros((3, 3))
    zero3_4 = np.zeros((3, 4))
    R1 = np.concatenate((eye3_3, zero3_3, zero3_4, zero3_3), 1)
    R2 = np.concatenate((zero3_3, eye3_3, zero3_4, zero3_3), 1)
    R3 = np.concatenate((zero3_3, zero3_3, dtheta_de, zero3_3), 1)
    R4 = np.concatenate((zero3_3, zero3_3, zero3_4, eye3_3), 1)
    dT_dxq = np.concatenate((R1, R2, R3, R4), 0)
    return dT_dxq

def dT_inv_d_x_q(phi, theta, psi):
    eps = .01
    dq_de = np.zeros((4, 3))
    e0_at_e, e1_at_e, e2_at_e, e3_at_e = Euler2Quaternion(phi, theta, psi)
    for i in range(0, 3):
        e_eps = np.array([[phi],
                          [theta],
                          [psi]])
        e_eps[i][0] += eps  # add eps to the ith state
        e0_at_e_eps, e1_at_e_eps, e2_at_e_eps, e3_at_e_eps = Euler2Quaternion(e_eps[0],e_eps[1],e_eps[2])
        d_theta_dxi = np.array([[(e0_at_e_eps - e0_at_e) / eps],
                                [(e1_at_e_eps - e1_at_e) / eps],
                                [(e2_at_e_eps - e2_at_e) / eps],
                                [(e3_at_e_eps - e3_at_e) / eps]])
        dq_de[:, i] = d_theta_dxi[:, 0, 0]
    eye3_3 = np.eye(3)
    zero3_3 = np.zeros((3, 3))
    zero4_3 = np.zeros((4, 3))
    R1 = np.concatenate((eye3_3, zero3_3, zero3_3, zero3_3), 1)
    R2 = np.concatenate((zero3_3, eye3_3, zero3_3, zero3_3), 1)
    R3 = np.concatenate((zero4_3, zero4_3, dq_de, zero4_3), 1)
    R4 = np.concatenate((zero3_3, zero3_3, zero3_3, eye3_3), 1)
    dT_inv_dxq = np.concatenate((R1, R2, R3, R4), 0)
    return dT_inv_dxq

def dT_dVa(mav, Va, delta_t):
    # returns the derivative of motor thrust with respect to Va
    return dThrust

def dT_ddelta_t(mav, Va, delta_t):
    # returns the derivative of motor thrust with respect to delta_t
    return dThrust