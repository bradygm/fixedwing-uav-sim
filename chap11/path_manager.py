import numpy as np
import sys
sys.path.append('..')
from chap11.dubins_parameters import dubins_parameters
from message_types.msg_path import msg_path

class path_manager:
    def __init__(self):
        # message sent to path follower
        self.path = msg_path()
        # pointers to previous, current, and next waypoints
        self.ptr_previous = 0
        self.ptr_current = 1
        self.ptr_next = 2
        # flag that request new waypoints from path planner
        self.flag_need_new_waypoints = True
        self.num_waypoints = 0
        self.halfspace_n = np.inf * np.ones((3,1))
        self.halfspace_r = np.inf * np.ones((3,1))
        # state of the manager state machine
        self.manager_state = 1
        # dubins path parameters
        self.dubins_path = dubins_parameters()

    def update(self, waypoints, radius, state):
        if self.path.flag_path_changed == True:
            self.path.flag_path_changed = False
        if waypoints.type == 'straight_line':
            self.line_manager(waypoints, state)
        elif waypoints.type == 'fillet':
            self.fillet_manager(waypoints, radius, state)
        elif waypoints.type == 'dubins':
            self.dubins_manager(waypoints, radius, state)
        else:
            print('Error in Path Manager: Undefined waypoint type.')
        if waypoints.flag_waypoints_changed == True:
            waypoints.flag_waypoints_changed = False
        return self.path

    def line_manager(self, waypoints, state):
        if waypoints.num_waypoints < 3:
            if waypoints.flag_waypoints_changed:
                self.path.flag = 'line'
                self.path.line_origin = waypoints.ned[:, 0]
                self.path.line_direction = waypoints.ned[:, 1]-waypoints.ned[:,0]
                self.path.line_direction = self.path.line_direction / np.linalg.norm(self.path.line_direction)
        else:
            if waypoints.flag_waypoints_changed:
                self.path.flag = 'line'
                self.initialize_pointers(waypoints)
                self.updateLineHalfSpace(waypoints)
            if self.inHalfSpace(np.array([state.pn, state.pe, -state.h]).T):
                self.increment_pointers()
                self.path.flag_path_changed = True
                self.updateLineHalfSpace(waypoints)


    def fillet_manager(self, waypoints, radius, state):
        if waypoints.num_waypoints < 3:
            if waypoints.flag_waypoints_changed:
                self.path.flag = 'line'
                self.path.line_origin = waypoints.ned[:, 0]
                self.path.line_direction = waypoints.ned[:, 1]-waypoints.ned[:,0]
                self.path.line_direction = self.path.line_direction / np.linalg.norm(self.path.line_direction)
        else:
            if waypoints.flag_waypoints_changed:
                self.path.flag = 'line'
                self.initialize_pointers(waypoints)
                self.manager_state = 1
                r = waypoints.ned[:, self.ptr_previous]
                q_prev = (waypoints.ned[:, self.ptr_current] - waypoints.ned[:, self.ptr_previous]) / \
                         np.linalg.norm(waypoints.ned[:, self.ptr_current] - waypoints.ned[:, self.ptr_previous])
                q = (waypoints.ned[:, self.ptr_next] - waypoints.ned[:, self.ptr_current]) / \
                    np.linalg.norm(waypoints.ned[:, self.ptr_next] - waypoints.ned[:, self.ptr_current])
                angle = np.arccos(-q_prev.T @ q)
                self.path.line_direction = q_prev
                self.path.line_origin = r
                z = waypoints.ned[:, self.ptr_current] - radius / np.tan(angle / 2) * q_prev
                self.halfspace_r = z
                self.halfspace_n = q_prev
            if self.inHalfSpace(np.array([state.pn, state.pe, -state.h]).T):
                if self.manager_state == 1:
                    self.manager_state = 2
                    self.path.type = 'orbit'
                    self.path.flag_path_changed = True
                    q_prev = (waypoints.ned[:, self.ptr_current] - waypoints.ned[:, self.ptr_previous]) / \
                             np.linalg.norm(waypoints.ned[:, self.ptr_current] - waypoints.ned[:, self.ptr_previous])
                    q = (waypoints.ned[:, self.ptr_next] - waypoints.ned[:, self.ptr_current]) / \
                        np.linalg.norm(waypoints.ned[:, self.ptr_next] - waypoints.ned[:, self.ptr_current])
                    angle = np.arccos(-q_prev.T @ q)
                    c = waypoints.ned[:, self.ptr_current] - (radius / np.sin(angle / 2.)) * (q_prev - q) / np.linalg.norm(
                        q_prev - q)
                    self.path.orbit_center = c
                    self.path.orbit_radius = radius
                    if np.sign(q_prev.item(0) * q.item(1) - q_prev.item(1) * q.item(0)) >= 0:
                        self.path.orbit_direction = 'CW'
                    else:
                        self.path.orbit_direction = 'CCW'
                    z = waypoints.ned[:, self.ptr_current] + radius / np.tan(angle / 2) * q
                    self.halfspace_r = z
                    self.halfspace_n = q
                else:
                    self.increment_pointers()
                    self.manager_state = 1
                    self.path.type = 'line'
                    self.path.flag_path_changed = True
                    r = waypoints.ned[:, self.ptr_previous]
                    q_prev = (waypoints.ned[:, self.ptr_current] - waypoints.ned[:, self.ptr_previous]) / \
                             np.linalg.norm(waypoints.ned[:, self.ptr_current] - waypoints.ned[:, self.ptr_previous])
                    q = (waypoints.ned[:, self.ptr_next] - waypoints.ned[:, self.ptr_current]) / \
                        np.linalg.norm(waypoints.ned[:, self.ptr_next] - waypoints.ned[:, self.ptr_current])
                    angle = np.arccos(-q_prev.T @ q)
                    self.path.line_direction = q_prev
                    self.path.line_origin = r
                    z = waypoints.ned[:, self.ptr_current] - radius / np.tan(angle / 2) * q_prev
                    self.halfspace_r = z
                    self.halfspace_n = q_prev



    def dubins_manager(self, waypoints, radius, state):
        a = 2

    def initialize_pointers(self, waypoints):
        self.num_waypoints = waypoints.num_waypoints
        self.ptr_previous = 0
        self.ptr_current = 1
        self.ptr_next = 2

    def increment_pointers(self):
        self.ptr_previous = self.ptr_current
        self.ptr_current = self.ptr_next
        self.ptr_next += 1
        if self.ptr_next == self.num_waypoints:
            self.ptr_next = 0

    def inHalfSpace(self, pos):
        if (pos-self.halfspace_r).T @ self.halfspace_n >= 0:
            return True
        else:
            return False

    def updateLineHalfSpace(self, waypoints):
        r = waypoints.ned[:, self.ptr_previous]
        q_prev = (waypoints.ned[:, self.ptr_current] - waypoints.ned[:, self.ptr_previous]) / np.linalg.norm(
            waypoints.ned[:, self.ptr_current] - waypoints.ned[:, self.ptr_previous])
        q = (waypoints.ned[:, self.ptr_next] - waypoints.ned[:, self.ptr_current]) / np.linalg.norm(
            waypoints.ned[:, self.ptr_next] - waypoints.ned[:, self.ptr_current])
        n = (q_prev + q) / np.linalg.norm(q_prev + q)
        self.halfspace_r = waypoints.ned[:, self.ptr_current]
        self.halfspace_n = n
        self.path.line_origin = r
        self.path.line_direction = q_prev