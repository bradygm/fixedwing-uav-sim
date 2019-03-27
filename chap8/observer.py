"""
observer
    - Beard & McLain, PUP, 2012
    - Last Update:
        3/2/2019 - RWB
"""
import sys
import numpy as np
sys.path.append('..')
# import parameters.control_parameters as CTRL
import parameters.simulation_parameters as SIM
import parameters.sensor_parameters as SENSOR
import parameters.aerosonde_parameters as MAV
sys.path.append('../tools')
# from angleConversions import Euler2Rotation

from message_types.msg_state import msg_state

class observer:
    def __init__(self, ts_control):
        # initialized estimated state message
        self.estimated_state = msg_state()
        # use alpha filters to low pass filter gyros and accels
        self.lpf_gyro_x = alpha_filter(alpha=0.5)
        self.lpf_gyro_y = alpha_filter(alpha=0.5)
        self.lpf_gyro_z = alpha_filter(alpha=0.5)
        self.lpf_accel_x = alpha_filter(alpha=0.5)
        self.lpf_accel_y = alpha_filter(alpha=0.5)
        self.lpf_accel_z = alpha_filter(alpha=0.5)
        # use alpha filters to low pass filter static and differential pressure
        self.lpf_static = alpha_filter(alpha=0.9)
        self.lpf_diff = alpha_filter(alpha=0.5)
        # ekf for phi and theta
        self.attitude_ekf = ekf_attitude()
        # ekf for pn, pe, Vg, chi, wn, we, psi
        self.position_ekf = ekf_position()

    def update(self, measurements):

        # estimates for p, q, r are low pass filter of gyro minus bias estimate
        self.estimated_state.p = self.lpf_gyro_x.update(measurements.gyro_x)
        self.estimated_state.q = self.lpf_gyro_y.update(measurements.gyro_y)
        self.estimated_state.r = self.lpf_gyro_z.update(measurements.gyro_z)

        # invert sensor model to get altitude and airspeed
        self.estimated_state.h = self.lpf_static.update(measurements.static_pressure)/(MAV.gravity*MAV.rho)
        self.estimated_state.Va = np.sqrt(2.*self.lpf_diff.update(measurements.diff_pressure)/MAV.rho)

        # estimate phi and theta with simple ekf
        # self.estimated_state.phi, self.estimated_state.theta = self.attitude_ekf.update(self.estimated_state, measurements)
        self.attitude_ekf.update(self.estimated_state, measurements)

        # # estimate pn, pe, Vg, chi, wn, we, psi
        self.position_ekf.update(self.estimated_state, measurements)
        #
        # # not estimating these
        self.estimated_state.alpha = self.estimated_state.theta
        self.estimated_state.beta = 0.0
        self.estimated_state.bx = 0.0
        self.estimated_state.by = 0.0
        self.estimated_state.bz = 0.0
        return self.estimated_state

class alpha_filter:
    # alpha filter implements a simple low pass filter
    # y[k] = alpha * y[k-1] + (1-alpha) * u[k]
    def __init__(self, alpha=0.5, y0=0.0):
        self.alpha = alpha  # filter parameter
        self.y = y0  # initial condition

    def update(self, u):
        self.y = self.y*self.alpha + (1-self.alpha)*u
        return self.y

class ekf_attitude:
    # implement continous-discrete EKF to estimate roll and pitch angles
    def __init__(self):
        self.Q = np.array([[.0000000000000001, 0.],
                           [0., .0000000000000001]])
        self.Q_gyro = np.array([[SENSOR.gyro_sigma**2, 0., 0.],
                                [0., SENSOR.gyro_sigma**2, 0.],
                                [0., 0., SENSOR.gyro_sigma**2]]) #3x3 on diagonal
        self.R_accel = np.array([[SENSOR.accel_sigma**2, 0., 0.],
                                 [0., SENSOR.accel_sigma**2, 0.],
                                 [0., 0., SENSOR.accel_sigma**2]])
        self.N = 10  # number of prediction step per sample
        self.xhat = np.array([[0., 0.]]).T # initial state: phi, theta Could initialize to real??
        self.P = np. array([[np.pi**2, 0.],
                            [0., np.pi**2]]) # initial P. What value??
        # self.P = np.array([[(np.pi/2)**2, 0.],
        #                   [0., (np.pi/2)**2]])  # initial P. What value??
        self.Ts = SIM.ts_control/self.N

    def update(self, state, measurement):
        self.propagate_model(state)
        self.measurement_update(state, measurement)
        state.phi = self.xhat.item(0) #should this return something??
        state.theta = self.xhat.item(1)

    def f(self, x, state):
        # system dynamics for propagation model: xdot = f(x, u)
        _f = np.array([[state.p + state.q*np.sin(x.item(0))*np.tan(x.item(1)) + state.r*np.cos(x.item(0))*np.tan(x.item(1))],
                       [state.q*np.cos(x.item(0)) - state.r*np.sin(x.item(0))]])
        #can switch out p for sensor ??
        return _f

    def h(self, x, state):
        # measurement model y
        _h = np.array([[state.q*state.Va*np.sin(x.item(1)) + MAV.gravity*np.sin(x.item(1))],
                       [state.r*state.Va*np.cos(x.item(1)) - state.p*state.Va*np.sin(x.item(1)) - MAV.gravity*np.cos(x.item(1))*np.sin(x.item(0))],
                       [-state.q*state.Va*np.cos(x.item(1)) - MAV.gravity*np.cos(x.item(1))*np.cos(x.item(0))]])
        return _h

    def propagate_model(self, state):
        # model propagation
        for i in range(0, self.N):
             # propagate model
            self.xhat = self.xhat + self.Ts*self.f(self.xhat, state)
            # compute Jacobian
            A = jacobian(self.f, self.xhat, state)
            # compute G matrix for gyro noise
            G = np.array([[1., np.sin(self.xhat.item(0))*np.tan(self.xhat.item(1)), np.cos(self.xhat.item(0))*np.tan(self.xhat.item(1))],
                          [0., np.cos(self.xhat.item(0)), -np.sin(self.xhat.item(0))]])
            # update P with continuous time model
            # self.P = self.P + self.Ts * (A @ self.P + self.P @ A.T + self.Q + G @ self.Q_gyro @ G.T)

            # convert to discrete time models
            A_d = np.eye(2) + A*self.Ts + A@A*self.Ts**2./2.
            G_d = self.Ts*G
            # update P with discrete time model
            self.P = A_d@self.P@A_d.T + G_d@self.Q_gyro@G_d.T + self.Q

    def measurement_update(self, state, measurement):
        # measurement updates
        threshold = 2.0
        h = self.h(self.xhat, state)
        C = jacobian(self.h, self.xhat, state)
        y = np.array([measurement.accel_x, measurement.accel_y, measurement.accel_z])
        for i in range(0, 3):
            if np.abs(y.item(i)-h.item(i)) < threshold:
                Ci = C[i,:]
                L = self.P@Ci.T/(self.R_accel[i,i] + Ci@self.P@Ci.T)
                # self.P = (np.eye(2) - L@Ci)@self.P
                self.P = (np.eye(2) - np.outer(L, Ci))@self.P

                change = L*(y.item(i) - h.item(i))
                # self.xhat = self.xhat + L*(y.item(i) - h.item(i))
                self.xhat = self.xhat + np.reshape(change, (2,1))

class ekf_position:
    # implement continous-discrete EKF to estimate pn, pe, Vg, chi, wn, we, psi
    def __init__(self):
        tuned_noise = .00000000000000001
        self.Q = np.array([[tuned_noise, 0., 0., 0., 0., 0., 0.],
                           [0., tuned_noise, 0., 0., 0., 0., 0.],
                           [0., 0., .001, 0., 0., 0., 0.],
                           [0., 0., 0., .01, 0., 0., 0.],
                           [0., 0., 0., 0., .0001, 0., 0.],
                           [0., 0., 0., 0., 0., .0001, 0.],
                           [0., 0., 0., 0., 0., 0., .001]])
        self.R = np.array([[SENSOR.gps_n_sigma**2, 0., 0., 0., 0., 0.],
                           [0., SENSOR.gps_e_sigma**2, 0., 0., 0., 0.],
                           [0., 0., SENSOR.gps_Vg_sigma**2/400, 0., 0., 0.],
                           [0., 0., 0., SENSOR.gps_course_sigma**2, 0., 0.]])
                           # [0., 0., 0., 0., SENSOR.gps_n_sigma**2, 0.],# What would this be??
                           # [0., 0., 0., 0., 0., SENSOR.gps_e_sigma**2]]) # What would this be??
        # self.R_pseudo = np.array([[SENSOR.static_pres_sigma**2, 0.],
        #                           [0., SENSOR.static_pres_sigma**2]]) #What would this be??
        self.R_pseudo = np.array([[1 ** 2, 0.],
                                  [0., 1 ** 2]])  # What would this be??
        self.N = 5  # number of prediction step per sample
        self.Ts = (SIM.ts_control / self.N)
        self.xhat = np.array([[0., 0., 25., 0., 0., 0., 0.]]).T #Could initialize better??
        self.P = np.array([[100**2, 0., 0., 0., 0., 0., 0.], #Start initialized??
                           [0., 100**2, 0., 0., 0., 0., 0.],
                           [0., 0., 1**2, 0., 0., 0., 0.],
                           [0., 0., 0., np.pi**2, 0., 0., 0.],
                           [0., 0., 0., 0., 1**2, 0., 0.],
                           [0., 0., 0., 0., 0., 1**2, 0.],
                           [0., 0., 0., 0., 0., 0., np.pi**2]])
        self.gps_n_old = 9999
        self.gps_e_old = 9999
        self.gps_Vg_old = 9999
        self.gps_course_old = 9999


    def update(self, state, measurement):
        self.propagate_model(state)
        self.measurement_update(state, measurement)
        state.pn = self.xhat.item(0)
        state.pe = self.xhat.item(1)
        state.Vg = self.xhat.item(2)
        state.chi = self.xhat.item(3)
        state.wn = self.xhat.item(4)
        state.we = self.xhat.item(5)
        state.psi = self.xhat.item(6)

    def f(self, x, state):
        # system dynamics for propagation model: xdot = f(x, u)
        #pn, pe, Vg, chi, wn, we, psi
        psi_dot = state.q*np.sin(state.phi)/np.cos(state.theta) + state.r*np.cos(state.phi)/np.cos(state.theta)
        _f = np.array([[x.item(2)*np.cos(x.item(3))],
                       [x.item(2)*np.sin(x.item(3))],
                       [((state.Va*np.cos(x.item(6)) + x.item(4))*(-state.Va*psi_dot*np.sin(x.item(6))) +
                         (state.Va*np.sin(x.item(6)) + x.item(5))*(state.Va*psi_dot*np.cos(x.item(6))))/x.item(2)],
                       [MAV.gravity*np.tan(state.phi)*np.cos(x.item(3)-x.item(6))/x.item(2)],
                       [0.],
                       [0.],
                       [psi_dot]])
        return _f

    def h_gps(self, x, state):
        # measurement model for gps measurements
        _h = np.array([[x.item(0)],
                       [x.item(1)],
                       [x.item(2)],
                       [x.item(3)]])
        return _h

    def h_pseudo(self, x, state):
        # measurement model for wind triangle pseudo measurement
        _h = np.array([[state.Va*np.cos(x.item(6)) + x.item(4) - x.item(2)*np.cos(x.item(3))],
                       [state.Va*np.sin(x.item(6)) + x.item(5) - x.item(2)*np.sin(x.item(3))]])
        return _h

    def propagate_model(self, state):
        # model propagation
        for i in range(0, self.N):
            # propagate model
            self.xhat = self.xhat + self.Ts*self.f(self.xhat, state)
            # compute Jacobian
            A = jacobian(self.f, self.xhat, state)
            # update P with continuous time model
            # self.P = self.P + self.Ts * (A @ self.P + self.P @ A.T + self.Q + G @ self.Q_gyro @ G.T)
            # convert to discrete time models
            A_d = np.eye(7) + A*self.Ts + (A@A*self.Ts**2.)/2.
            # update P with discrete time model
            self.P = A_d@self.P@A_d.T + self.Q*self.Ts**2

    def measurement_update(self, state, measurement):
        # always update based on wind triangle pseudu measurement
        h = self.h_pseudo(self.xhat, state)
        C = jacobian(self.h_pseudo, self.xhat, state)
        y = np.array([[0, 0]])
        for i in range(0, 2):
            Ci = C[i, :]
            L = self.P@Ci.T/(self.R_pseudo[i, i] + Ci@self.P@Ci.T)
            self.P = (np.eye(7) - np.outer(L, Ci))@self.P
            change = L * (y.item(i) - h.item(i))
            self.xhat = self.xhat + np.reshape(change, (7, 1))

        # only update GPS when one of the signals changes
        if (measurement.gps_n != self.gps_n_old) \
            or (measurement.gps_e != self.gps_e_old) \
            or (measurement.gps_Vg != self.gps_Vg_old) \
            or (measurement.gps_course != self.gps_course_old):

            h = self.h_gps(self.xhat, state)
            C = jacobian(self.h_gps, self.xhat, state)
            y = np.array([measurement.gps_n, measurement.gps_e, measurement.gps_Vg, measurement.gps_course])
            for i in range(0, 4):
                Ci = C[i, :]
                L = self.P@Ci.T/(self.R[i, i] + Ci@self.P@Ci.T)
                self.P = (np.eye(7) - np.outer(L, Ci))@self.P
                change = L * (y.item(i) - h.item(i))
                self.xhat = self.xhat + np.reshape(change, (7, 1))
            # update stored GPS signals
            self.gps_n_old = measurement.gps_n
            self.gps_e_old = measurement.gps_e
            self.gps_Vg_old = measurement.gps_Vg
            self.gps_course_old = measurement.gps_course
        self.xhat[3] = self.wrap(self.xhat[3], 0)
        self.xhat[6] = self.wrap(self.xhat[6], 0)


    def wrap(self, chi_c, chi):
        while chi_c-chi > np.pi:
            chi_c = chi_c - 2.0 * np.pi
        while chi_c-chi < -np.pi:
            chi_c = chi_c + 2.0 * np.pi
        return chi_c

def jacobian(fun, x, state):
    # compute jacobian of fun with respect to x
    f = fun(x, state)
    m = f.shape[0]
    n = x.shape[0]
    eps = 0.01  # deviation
    J = np.zeros((m, n))
    for i in range(0, n):
        x_eps = np.copy(x)
        x_eps[i][0] += eps
        f_eps = fun(x_eps, state)
        df = (f_eps - f) / eps
        J[:, i] = df[:, 0]
    return J