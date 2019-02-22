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
pitch_kp = 0.
pitch_kd = 0.
K_theta_DC = 0.

#----------altitude loop-------------
altitude_kp = 0.
altitude_ki = 0.
altitude_zone = 0.

#---------airspeed hold using throttle---------------
airspeed_throttle_kp = 0.
airspeed_throttle_ki = 0.
