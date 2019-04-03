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
                self.updateFilletHalfSpace(waypoints, radius, self.manager_state)
            if self.inHalfSpace(np.array([state.pn, state.pe, -state.h]).T):
                if self.manager_state == 1:
                    self.manager_state = 2
                    self.path.type = 'orbit'
                    self.path.flag_path_changed = True
                    self.updateFilletHalfSpace(waypoints, radius, self.manager_state)
                else:
                    self.increment_pointers()
                    self.manager_state = 1
                    self.path.type = 'line'
                    self.path.flag_path_changed = True
                    self.updateFilletHalfSpace(waypoints, radius, self.manager_state)


    def dubins_manager(self, waypoints, radius, state):
        if waypoints.num_waypoints < 3:
            print("Must have at least 3 waypoints")
        else:
            if waypoints.flag_waypoints_changed:
                self.initialize_pointers(waypoints)
            self.dubins_path.update(waypoints.ned[:, self.ptr_previous], waypoints.course.item(self.ptr_previous),
                                    waypoints.ned[:, self.ptr_current], waypoints.course.item(self.ptr_current), radius)
            if self.manager_state == 1:
                self.path.type = 'orbit'
                self.path.flag_path_changed = True
                self.path.orbit_center = self.dubins_path.center_s
                self.path.orbit_radius = self.dubins_path.radius
                self.orbitDirection(self.dubins_path.dir_s)
                self.halfspace_r = self.dubins_path.r1
                self.halfspace_n = -self.dubins_path.n1
                if self.inHalfSpace(np.array([state.pn, state.pe, -state.h]).T):
                    self.manager_state = 2
                    self.halfspace_r = self.dubins_path.r1
                    self.halfspace_n = self.dubins_path.n1
            elif self.manager_state == 2:
                if self.inHalfSpace(np.array([state.pn, state.pe, -state.h]).T):
                    self.manager_state = 3
            elif self.manager_state == 3:
                self.path.type = 'line'
                self.path.flag_path_changed = True
                self.path.line_origin = self.dubins_path.r1
                self.path.line_direction = self.dubins_path.n1
                self.halfspace_r = self.dubins_path.r2
                self.halfspace_n = self.dubins_path.n1
                if self.inHalfSpace(np.array([state.pn, state.pe, -state.h]).T):
                    self.manager_state = 4
            elif self.manager_state == 4:
                self.path.type = 'orbit'
                self.path.flag_path_changed = True
                self.path.orbit_center = self.dubins_path.center_e
                self.path.orbit_radius = self.dubins_path.radius
                self.orbitDirection(self.dubins_path.dir_e)
                self.halfspace_r = self.dubins_path.r3
                self.halfspace_n = -self.dubins_path.n3
                if self.inHalfSpace(np.array([state.pn, state.pe, -state.h]).T):
                    self.manager_state = 5
                    self.halfspace_r = self.dubins_path.r3
                    self.halfspace_n = self.dubins_path.n3
            elif self.manager_state == 5:
                if self.inHalfSpace(np.array([state.pn, state.pe, -state.h]).T):
                    self.manager_state = 1
                    self.increment_pointers()
                    self.dubins_path.update(waypoints.ned[:, self.ptr_previous],
                                            waypoints.course.item(self.ptr_previous),
                                            waypoints.ned[:, self.ptr_current], waypoints.course.item(self.ptr_current),
                                            radius)
            else:
                print("Error in manager state")






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

    def updateFilletHalfSpace(self, waypoints, radius, manager_state):
        q_prev = (waypoints.ned[:, self.ptr_current] - waypoints.ned[:, self.ptr_previous]) / \
                 np.linalg.norm(waypoints.ned[:, self.ptr_current] - waypoints.ned[:, self.ptr_previous])
        q = (waypoints.ned[:, self.ptr_next] - waypoints.ned[:, self.ptr_current]) / \
            np.linalg.norm(waypoints.ned[:, self.ptr_next] - waypoints.ned[:, self.ptr_current])
        angle = np.arccos(-q_prev.T @ q)
        if manager_state == 2:
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
            self.path.line_direction = q_prev
            self.path.line_origin = waypoints.ned[:, self.ptr_previous]
            z = waypoints.ned[:, self.ptr_current] - radius / np.tan(angle / 2) * q_prev
            self.halfspace_r = z
            self.halfspace_n = q_prev

    def orbitDirection(self, orbitD):
        if orbitD == 1:
            self.path.orbit_direction = 'CW'
        elif orbitD == -1:
            self.path.orbit_direction = 'CCW'
        else:
            print("error in orbitDirection")