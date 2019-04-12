# path planner for mavsim_python
#
# mavsim_python
#     - Beard & McLain, PUP, 2012
#     - Last updated:
#         4/3/2019 - BGM
import numpy as np
import sys
sys.path.append('..')
from message_types.msg_waypoints import msg_waypoints
from planRRT import planRRT
from planRRTDubins import planRRTDubins

class path_planner:
    def __init__(self):
        # waypoints definition
        self.waypoints = msg_waypoints()
        self.rrt = planRRT(map)
        self.rrtDubins = planRRTDubins(map)

    def update(self, map, state, PLAN):
        # this flag is set for one time step to signal a redraw in the viewer
        # planner_flag = 1  # return simple waypoint path
        # planner_flag = 2  # return dubins waypoint path
        # planner_flag = 3  # plan path through city using straight-line RRT
        planner_flag = 4  # plan path through city using dubins RRT
        if planner_flag == 1:
            self.waypoints.type = 'fillet'
            self.waypoints.num_waypoints = 4
            Va = 25
            self.waypoints.ned[:, 0:self.waypoints.num_waypoints] \
                = np.array([[0, 0, -100],
                            [1000, 0, -100],
                            [0, 1000, -100],
                            [1000, 1000, -100]]).T
            self.waypoints.airspeed[:, 0:self.waypoints.num_waypoints] \
                = np.array([[Va, Va, Va, Va]])
        elif planner_flag == 2:
            self.waypoints.type = 'dubins'
            self.waypoints.num_waypoints = 4
            Va = 25
            self.waypoints.ned[:, 0:self.waypoints.num_waypoints] \
                = np.array([[0, 0, -100],
                            [1000, 0, -100],
                            [0, 1000, -100],
                            [1000, 1000, -100]]).T
            self.waypoints.airspeed[:, 0:self.waypoints.num_waypoints] \
                = np.array([[Va, Va, Va, Va]])
            self.waypoints.course[:, 0:self.waypoints.num_waypoints] \
                = np.array([[np.radians(0),
                             np.radians(45),
                             np.radians(45),
                             np.radians(-135)]])
        elif planner_flag == 3:
            self.waypoints.type = 'fillet'
            self.waypoints.num_waypoints = 0
            Va = 25
            primaryWaypoints = np.array([[0., 0., -100.],
                                        [1000., 0., -100.],
                                        [0., 1000., -100.],
                                        [1200., 1200., -100.]]).T
            primaryWaypointsAirspeed = np.array([[Va, Va, Va, Va]])
            for i in range(0, np.size(primaryWaypoints,1)):
                # current configuration vector format: N, E, D, Va
                if i == 0:
                    wpp_start = np.array([state.pn,
                                          state.pe,
                                          -state.h,
                                          state.Va])
                else:
                    wpp_start = np.array([primaryWaypoints[0,i-1],
                                          primaryWaypoints[1,i-1],
                                          primaryWaypoints[2,i-1],
                                          primaryWaypointsAirspeed.item(i-1)])
                wpp_end = np.array([primaryWaypoints[0, i],
                                      primaryWaypoints[1, i],
                                      primaryWaypoints[2, i],
                                      primaryWaypointsAirspeed.item(i)])
                waypoints = self.rrt.planPath(wpp_start, wpp_end, map)
                numNew = waypoints.num_waypoints
                numOld = self.waypoints.num_waypoints
                self.waypoints.ned[:, numOld:numOld+numNew] = waypoints.ned[:, 0:numNew]
                self.waypoints.airspeed[numOld:numOld+numNew] = Va
                self.waypoints.num_waypoints = numNew + numOld
        elif planner_flag == 4:
            self.waypoints.type = 'dubins'
            self.waypoints.num_waypoints = 0
            Va = 25
            numberWaypoints = 4
            primaryWaypoints = np.array([[0., 0., -100.],
                                         [2000., 0., -100.],
                                         [0., 1200., -100.],
                                         [3000., 3000., -100.]]).T
            primaryWaypointsAirspeed = np.array([[Va, Va, Va, Va]])
            primaryCourseAngles = np.array([[np.radians(0),
                                             np.radians(45),
                                             np.radians(45),
                                             np.radians(-135)]])
            # numberWaypoints = 2
            # primaryWaypoints = np.array([[0., 0., -100.],
            #                              [1000., 0., -100.]]).T
            # primaryWaypointsAirspeed = np.array([[Va, Va]])
            # primaryCourseAngles = np.array([[np.radians(0),
            #                                  np.radians(45)]])
            for i in range(0, numberWaypoints):
                # current configuration vector format: N, E, D, Va
                if i == 0 and np.sqrt((state.pn - primaryWaypoints[0,0])**2 + (state.pe - primaryWaypoints[1,0])**2) > PLAN.R_min:
                    wpp_start = np.array([state.pn,
                                          state.pe,
                                          -state.h,
                                          state.chi,
                                          state.Va])
                    self.waypoints.ned[:, self.waypoints.num_waypoints] = wpp_start[0:3]
                    self.waypoints.course[:, self.waypoints.num_waypoints] = wpp_start.item(3)
                    self.waypoints.airspeed[:, self.waypoints.num_waypoints] = wpp_start.item(4)
                    self.waypoints.num_waypoints += 1

                elif i == 0:
                    self.waypoints.ned[:, self.waypoints.num_waypoints] = np.array([primaryWaypoints[0, 0],
                                                                                      primaryWaypoints[1, 0],
                                                                                      primaryWaypoints[2, 0]])
                    self.waypoints.course[:, self.waypoints.num_waypoints] = primaryCourseAngles.item(0)
                    self.waypoints.airspeed[:, self.waypoints.num_waypoints] = primaryWaypointsAirspeed.item(0)
                    self.waypoints.num_waypoints += 1
                    continue
                else:
                    wpp_start = np.array([primaryWaypoints[0, i - 1],
                                          primaryWaypoints[1, i - 1],
                                          primaryWaypoints[2, i - 1],
                                          primaryCourseAngles.item(i-1),
                                          primaryWaypointsAirspeed.item(i-1)])
                wpp_end = np.array([primaryWaypoints[0, i],
                                    primaryWaypoints[1, i],
                                    primaryWaypoints[2, i],
                                    primaryCourseAngles.item(i),
                                    primaryWaypointsAirspeed.item(i)])
                waypoints = self.rrtDubins.planPath(wpp_start, wpp_end, PLAN.R_min, map)
                numNew = waypoints.num_waypoints-1
                numOld = self.waypoints.num_waypoints
                if numNew > 1:
                    self.waypoints.ned[:, numOld:numOld + numNew] = waypoints.ned[:, 1:numNew+1]
                    self.waypoints.course[:,numOld:numOld + numNew] = waypoints.course[:,1:numNew+1]
                    self.waypoints.airspeed[:, numOld:(numOld + numNew)] = wpp_end.item(4) * np.ones((1, numNew))
                else:
                    self.waypoints.ned[:, numOld] = waypoints.ned[:, 1]
                    self.waypoints.course[:, numOld] = waypoints.course[:, 1]
                    self.waypoints.airspeed[:, numOld] = wpp_end.item(4) * np.ones((1, numNew))
                self.waypoints.num_waypoints = numNew + numOld
        else:
            print("Error in Path Planner: Undefined planner type.")

        return self.waypoints
