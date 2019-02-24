"""
autopilot block for mavsim_python
    - Beard & McLain, PUP, 2012
    - Last Update:
        2/6/2019 - RWB
"""
import sys
import numpy as np
sys.path.append('..')
import parameters.control_parameters as AP
sys.path.append('../tools')
from transfer_function import transfer_function
from chap6.pid_control import pid_control, pi_control, pd_control_with_rate
from message_types.msg_state import msg_state


class autopilot:
    def __init__(self, ts_control):
        # instantiate lateral controllers
        self.roll_from_aileron = pd_control_with_rate(
                        kp=AP.roll_kp,
                        kd=AP.roll_kd,
                        limit=np.radians(45))
        self.course_from_roll = pi_control(
                        kp=AP.course_kp,
                        ki=AP.course_ki,
                        Ts=ts_control,
                        limit=np.radians(30))
        self.sideslip_from_rudder = pi_control(
                        kp=AP.sideslip_kp,
                        ki=AP.sideslip_ki,
                        Ts=ts_control,
                        limit=np.radians(45))
        self.yaw_damper = transfer_function(
                        num=np.array([[AP.yaw_damper_kp, 0]]),
                        den=np.array([[1, 1 / AP.yaw_damper_tau_r]]),
                        Ts=ts_control)

        # instantiate longitudinal controllers
        self.pitch_from_elevator = pd_control_with_rate(
                        kp=AP.pitch_kp,
                        kd=AP.pitch_kd,
                        limit=np.radians(45))
        self.altitude_from_pitch = pi_control(
                        kp=AP.altitude_kp,
                        ki=AP.altitude_ki,
                        Ts=ts_control,
                        limit=np.radians(30))
        self.airspeed_from_throttle = pi_control(
                        kp=AP.airspeed_throttle_kp,
                        ki=AP.airspeed_throttle_ki,
                        Ts=ts_control,
                        limit=1.0)
        self.commanded_state = msg_state()

    def update(self, cmd, state):

        # lateral autopilot
        course_wrapped = self.wrap(state.chi,cmd.course_command)
        phi_c = self.course_from_roll.update(course_wrapped, state.chi)
        # phi_c = np.radians(0)
        delta_a = self.roll_from_aileron.update(phi_c, state.phi, state.p)
        delta_r = self.yaw_damper.update(state.r)

        # longitudinal autopilot
        h_c = cmd.altitude_command
        h_c_filtered = self.saturate(h_c, state.h - AP.altitude_hold_zone, state.h + AP.altitude_hold_zone)
        theta_c = self.altitude_from_pitch.update(h_c_filtered,state.h)
        # theta_c = np.radians(15)
        delta_e = self.pitch_from_elevator.update(theta_c, state.theta, state.q)
        delta_t = self.airspeed_from_throttle.update(cmd.airspeed_command, state.Va)

        # construct output and commanded states
        #(delta_a, delta_e, delta_r, delta_t)
        delta = np.array([[delta_a], [delta_e], [delta_r], [delta_t]])
        self.commanded_state.h = cmd.altitude_command
        self.commanded_state.Va = cmd.airspeed_command
        self.commanded_state.phi = phi_c
        self.commanded_state.theta = theta_c
        self.commanded_state.chi = cmd.course_command
        return delta, self.commanded_state

    def saturate(self, input, low_limit, up_limit):
        if input <= low_limit:
            output = low_limit
        elif input >= up_limit:
            output = up_limit
        else:
            output = input
        return output

    def wrap(self, current, input):
        if (input - current) > np.pi:
            input -= 2*np.pi
        elif (input - current) < -np.pi:
            input += 2*np.pi
        return input