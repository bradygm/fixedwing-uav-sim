import sys
sys.path.append('..')
import numpy as np
import chap5.tf_coefficients as TF

gravity = -9.8
sigma = .05
Va0 = 25

#----------roll loop-------------
zeta = .707
roll_kp = np.radians(45)/np.radians(45)
roll_omega = np.sqrt(roll_kp*TF.T_phi_delta_a_a2)
roll_kd = (2.*zeta*roll_omega - TF.T_phi_delta_a_a1)/TF.T_phi_delta_a_a2
# roll_kp = .3
# roll_kd = -.7

#----------course loop-------------
course_kp = 0.
course_ki = 0.

#----------sideslip loop-------------
sideslip_ki = 0.
sideslip_kp = 0.

# ----------yaw damper-------------
yaw_damper_tau_r = 0.
yaw_damper_kp = 0.

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
