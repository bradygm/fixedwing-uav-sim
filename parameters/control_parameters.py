import sys
sys.path.append('..')
import numpy as np
import chap5.tf_coefficients as TF

gravity = 9.8
sigma = .05
Va0 = 25

#----------roll loop-------------
omega_n_phi = 9.2
zeta_phi = .707
roll_kp = omega_n_phi**2/TF.T_phi_delta_a_a2
# roll_omega = np.sqrt(roll_kp*TF.T_phi_delta_a_a2)
roll_kd = (2.*zeta_phi*omega_n_phi - TF.T_phi_delta_a_a1)/TF.T_phi_delta_a_a2
# roll_kp = .3
roll_kd = -roll_kd*2.

#----------course loop-------------
separation_factor_chi = 8
omega_n_chi = omega_n_phi/separation_factor_chi
zeta_chi = 5
course_kp = 2.*zeta_chi*omega_n_chi*Va0/gravity #negativ gravity??
course_ki = omega_n_chi**2*Va0/gravity
course_ki = course_ki*3 #This could use more tuning. Once the output isn't railed, the integrator has problems.

#----------sideslip loop-------------
sideslip_ki = 0.
sideslip_kp = 0.

# ----------yaw damper-------------
yaw_damper_tau_r = 0.5
yaw_damper_kp = .75

#----------pitch loop-------------
omega_n_theta = 5
zeta_theta = .707
pitch_kp = -(omega_n_theta**2 - TF.T_theta_delta_e_a2)/TF.T_theta_delta_e_a3 #why negative??
pitch_kd = 4.*(2.*zeta_theta*omega_n_theta - TF.T_theta_delta_e_a1)/TF.T_theta_delta_e_a3
K_theta_DC = pitch_kp*TF.T_theta_delta_e_a3/omega_n_theta**2

#----------altitude loop-------------
separation_factor_h = 12
omega_n_h = omega_n_theta/separation_factor_h
zeta_h = 2.
altitude_kp = 2.*zeta_h*omega_n_h/(K_theta_DC*Va0)
altitude_ki = omega_n_h**2/(K_theta_DC*Va0)
altitude_zone = 0.

#---------airspeed hold using throttle---------------
omega_n_va = 8
zeta_va = 3
airspeed_throttle_kp = omega_n_va**2/TF.T_Va_delta_t_a2[0]
airspeed_throttle_ki = (2.*zeta_va*omega_n_va - TF.T_Va_delta_t_a1[0])/TF.T_Va_delta_t_a2[0]


altitude_hold_zone = 50 #Not sure what is a good number for this. Also, on slides, the pi controllers have a 0 and P
