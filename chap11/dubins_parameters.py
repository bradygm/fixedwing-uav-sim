# dubins_parameters
#   - Dubins parameters that define path between two configurations
#
# mavsim_matlab 
#     - Beard & McLain, PUP, 2012
#     - Update history:  
#         3/26/2019 - RWB

import numpy as np
import sys
sys.path.append('..')


class dubins_parameters:
    def __init__(self):
        self.p_s = np.inf*np.ones((3,1))  # the start position in re^3
        self.chi_s = np.inf  # the start course angle
        self.p_e = np.inf*np.ones((3,1))  # the end position in re^3
        self.chi_e = np.inf  # the end course angle
        self.radius = np.inf  # turn radius
        self.length = np.inf  # length of the Dubins path
        self.center_s = np.inf*np.ones((3,1))  # center of the start circle
        self.dir_s = np.inf  # direction of the start circle
        self.center_e = np.inf*np.ones((3,1))  # center of the end circle
        self.dir_e = np.inf  # direction of the end circle
        self.r1 = np.inf*np.ones((3,1))  # vector in re^3 defining half plane H1
        self.r2 = np.inf*np.ones((3,1))  # vector in re^3 defining position of half plane H2
        self.r3 = np.inf*np.ones((3,1))  # vector in re^3 defining position of half plane H3
        self.n1 = np.inf*np.ones((3,1))  # unit vector in re^3 along straight line path
        self.n3 = np.inf*np.ones((3,1))  # unit vector defining direction of half plane H3

    def update(self, ps, chis, pe, chie, R):
        ell = np.linalg.norm(ps - pe)
        if ell < 2 * R:
            print('Error in Dubins Parameters: The distance between nodes must be larger than 2R.')
        else:
            e1 = np.array([1., 0., 0.]).T
            self.p_s = ps
            self.chi_s = chis
            self.p_e = pe
            self.chi_e = chie
            self.radius = R
            # c_rs = ps + R * rotz(np.pi / 2) @ np.array([np.cos(chis), np.sin(chis), 0.]).T
            # c_ls = ps + R * rotz(-np.pi / 2) @ np.array([np.cos(chis), np.sin(chis), 0.]).T
            # c_re = pe + R * rotz(np.pi / 2) @ np.array([np.cos(chie), np.sin(chie), 0.]).T
            # c_le = pe + R * rotz(-np.pi / 2) @ np.array([np.cos(chie), np.sin(chie), 0.]).T
            c_rs = ps + R * np.array([np.cos(chis + np.pi/2), np.sin(chis + np.pi/2), 0.]).T
            c_ls = ps + R * np.array([np.cos(chis - np.pi/2), np.sin(chis - np.pi/2), 0.]).T
            c_re = pe + R * np.array([np.cos(chie + np.pi/2), np.sin(chie + np.pi/2), 0.]).T
            c_le = pe + R * np.array([np.cos(chie - np.pi/2), np.sin(chie - np.pi/2), 0.]).T
            theta = mod(np.arctan2(c_re.item(1)-c_rs.item(1), c_re.item(0)-c_rs.item(0)))
            L1 = np.linalg.norm(c_rs - c_re) + \
                 R*mod(2*np.pi + mod(theta - np.pi/2) - mod(chis - np.pi/2)) + \
                 R*mod(2*np.pi + mod(chie - np.pi/2) - mod(theta - np.pi/2))
            ell = np.linalg.norm(c_le - c_rs)
            theta = mod(np.arctan2(c_le.item(1) - c_rs.item(1), c_le.item(0) - c_rs.item(0)))
            theta2 = mod(theta - np.pi / 2 + np.arcsin(2 * R / ell))
            L2 = np.sqrt(ell**2 - 4.*R**2) + \
                 R*mod(2*np.pi + mod(theta2) - mod(chis - np.pi/2)) + \
                 R*mod(2*np.pi + mod(theta2 + np.pi) - mod(chie + np.pi/2))
            ell = np.linalg.norm(c_re - c_ls)
            theta = mod(np.arctan2(c_re.item(1) - c_ls.item(1), c_re.item(0) - c_ls.item(0)))
            theta2 = mod(np.arccos(2 * R / ell))
            L3 = np.sqrt(ell**2 - 4*R**2) + \
                 R*mod(2*np.pi + mod(chis + np.pi/2) - mod(theta + theta2)) + \
                 R*mod(2*np.pi + mod(chie - np.pi/2) - mod(theta + theta2 - np.pi))
            theta = mod(np.arctan2(c_le.item(1) - c_ls.item(1), c_le.item(0) - c_ls.item(0)))
            L4 = np.linalg.norm(c_ls - c_le) + \
                 R*mod(2*np.pi + mod(chis + np.pi/2) - mod(theta + np.pi/2)) + \
                 R*mod(2*np.pi + mod(theta + np.pi/2) - mod(chie + np.pi/2))
            minIndex = np.argmin([L1, L2, L3, L4])
            if minIndex == 0:
                self.length = L1
                self.center_s = c_rs
                self.dir_s = 1
                self.center_e = c_re
                self.dir_e = 1
                self.n1 = (c_re - c_rs)/np.linalg.norm(c_re - c_rs)
                self.r1 = c_rs + R*rotz(-np.pi/2)@self.n1
                self.r2 = c_re + R*rotz(-np.pi/2)@self.n1
            elif minIndex == 1:
                self.length = L2
                self.center_s = c_rs
                self.dir_s = 1
                self.center_e = c_le
                self.dir_e = -1
                ell = np.linalg.norm(c_le - c_rs)
                theta = np.arctan2(c_le.item(1) - c_rs.item(1), c_le.item(0) - c_rs.item(0))
                theta2 = theta - np.pi / 2 + np.arcsin(2 * R / ell)
                self.n1 = rotz(theta2+np.pi/2)@e1
                self.r1 = c_rs + R*rotz(theta2)@e1
                self.r2 = c_le + R*rotz(theta2 + np.pi)@e1
            elif minIndex == 2:
                self.length = L3
                self.center_s = c_ls
                self.dir_s = -1
                self.center_e = c_re
                self.dir_e = 1
                ell = np.linalg.norm(c_re - c_ls)
                theta = np.arctan2(c_re.item(1) - c_ls.item(1), c_re.item(0) - c_ls.item(0))
                theta2 = np.arccos(2 * R / ell)
                self.n1 = rotz(theta + theta2 -np.pi/2)@e1
                self.r1 = c_ls + R*rotz(theta + theta2)@e1
                self.r2 = c_re + R*rotz(theta + theta2 - np.pi)@e1
            elif minIndex == 3:
                self.length = L4
                self.center_s = c_ls
                self.dir_s = -1
                self.center_e = c_le
                self.dir_e = -1
                self.n1 = (c_le - c_ls)/np.linalg.norm(c_le - c_ls)
                self.r1 = c_ls + R*rotz(np.pi/2)@self.n1
                self.r2 = c_le + R*rotz(np.pi/2)@self.n1
            else:
                print("Error in finding minimum length")
            self.r3 = pe
            self.n3 = rotz(chie)@e1


def rotz(theta):
    return np.array([[np.cos(theta), -np.sin(theta), 0],
                    [np.sin(theta), np.cos(theta), 0],
                    [0, 0, 1]])


def mod(x):
    # make x between 0 and 2*pi
    while x < 0:
        x += 2*np.pi
    while x > 2*np.pi:
        x -= 2*np.pi
    return x


