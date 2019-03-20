import numpy as np
from math import sin, cos, atan, atan2
import sys

sys.path.append('..')
from message_types.msg_autopilot import msg_autopilot

class path_follower:
    def __init__(self):
        self.chi_inf = np.pi/4  # approach angle for large distance from straight-line path
        self.k_path = .1  # proportional gain for straight-line path following
        self.k_orbit = 7.3  # proportional gain for orbit following, why so high??
        self.gravity = 9.8
        self.autopilot_commands = msg_autopilot()  # message sent to autopilot

    def update(self, path, state):
        if path.flag=='line':
            self._follow_straight_line(path, state)
        elif path.flag=='orbit':
            self._follow_orbit(path, state)
        return self.autopilot_commands

    def _follow_straight_line(self, path, state):
        #calculate the commanded height
        chi_q = atan2(path.line_direction.item(1),path.line_direction.item(0)) #chi of desired path
        R_i_p = np.array([[cos(chi_q), sin(chi_q), 0.], #rotation to frame of the desired path
                          [-sin(chi_q), cos(chi_q), 0.],
                          [0., 0., 1.,]])
        e_p_i = np.array([(state.pn - path.line_origin.item(0)), #path error in inertial frame
                          (state.pe - path.line_origin.item(1)),
                          (-state.h - path.line_origin.item(2))])
        e_p = R_i_p@e_p_i #path error in path frame
        cross_q_k = np.cross(path.line_direction.T, np.array([0., 0., -1.]))
        cross_q_k = np.reshape(cross_q_k, (3))
        n = cross_q_k/np.linalg.norm(cross_q_k) #normal vector to plane q ki
        s = e_p_i - (e_p_i@n)*n #projection of position onto q ki plane
        h_d = -path.line_origin.item(2) + np.sqrt(s.item(0)**2 + s.item(1)**2)*(-path.line_direction.item(2)/(np.sqrt(path.line_direction.item(0)**2
                                             + path.line_direction.item(1)**2)))

        #calculate the commanded course
        chi_q_wrapped = self._wrap(chi_q, state.chi)
        e_p_y = e_p.item(1)
        chi_command = chi_q_wrapped - self.chi_inf*2./np.pi*atan(self.k_path*e_p_y)



        self.autopilot_commands.airspeed_command = path.airspeed
        self.autopilot_commands.course_command = chi_command
        self.autopilot_commands.altitude_command = h_d
        self.autopilot_commands.phi_feedforward = 0.

    def _follow_orbit(self, path, state):
        d = np.sqrt((state.pn - path.orbit_center.item(0))**2 + (state.pe - path.orbit_center.item(1))**2)
        loopyThang = atan2(state.pe - path.orbit_center.item(1),state.pn - path.orbit_center.item(0))
        loopyThang_wrap = self._wrap(loopyThang, state.chi)
        if path.orbit_direction=='CW':
            lambda_direction = 1
        else:
            lambda_direction = -1
        crossTrackError = (d - path.orbit_radius)/path.orbit_radius
        chi_command = loopyThang_wrap + lambda_direction*(np.pi/2. + atan(self.k_orbit*(crossTrackError)))

        #Solve for feedforward term
        wd = 0. #Assume wind in down direction is zero
        a1 = state.wn*cos(state.chi) + state.we*sin(state.chi)
        a2 = np.sqrt(state.Va**2 - (state.wn*sin(state.chi) - state.we*cos(state.chi))**2 - wd**2)
        if state.Va == 0:
            state.Va = .001
        b1 = self.gravity*path.orbit_radius*np.sqrt((state.Va**2-(state.wn*sin(state.chi) - state.we*cos(state.chi))**2 - wd**2)/(state.Va**2 - wd**2))

        self.autopilot_commands.airspeed_command = path.airspeed
        self.autopilot_commands.course_command = chi_command
        self.autopilot_commands.altitude_command = -path.orbit_center.item(2)
        # if crossTrackError < 10.25:
        # self.autopilot_commands.phi_feedforward = atan(state.Va**2/(self.gravity*path.orbit_radius))
        self.autopilot_commands.phi_feedforward = (a1 + a2)**2/b1
        # else:
        # self.autopilot_commands.phi_feedforward = 0.



    def _wrap(self, chi_c, chi):
        while chi_c-chi > np.pi:
            chi_c = chi_c - 2.0 * np.pi
        while chi_c-chi < -np.pi:
            chi_c = chi_c + 2.0 * np.pi
        return chi_c

