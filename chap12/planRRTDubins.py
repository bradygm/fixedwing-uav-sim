import numpy as np
from message_types.msg_waypoints import msg_waypoints
from chap11.dubins_parameters import dubins_parameters



class planRRTDubins():
    def __init__(self, map):
        self.segmentLength = 300 # standard length of path segments
        self.pointsAlongPathSpacing = .1
        self.clearance = 20
        # np.random.seed(1111)  # For Debugging

    def planPath(self, wpp_start, wpp_end, R_min, map):

        self.segmentLength = 3.5*R_min #2.5 ??

        # desired down position is down position of end node
        pd = wpp_end.item(2)

        # specify start and end nodes from wpp_start and wpp_end
        # format: N, E, D, chi, cost, parentIndex, connectsToGoalFlag,
        start_node = np.array([wpp_start.item(0), wpp_start.item(1), pd, self.mod(wpp_start.item(3)), 0, -1, 0])
        end_node = np.array([wpp_end.item(0), wpp_end.item(1), pd, self.mod(wpp_end.item(3)), 0, 0, 0])

        # establish tree starting with the start node
        tree = np.array([start_node])

        # check to see if start_node connects directly to end_node
        if ((np.linalg.norm(start_node[0:3] - end_node[0:3]) < self.segmentLength)
                and np.linalg.norm(start_node[0:3] - end_node[0:3])>=3.*R_min
                and not self.collision(start_node, end_node, map, R_min)):
            waypointsPath = msg_waypoints()
            waypointsPath.ned[:,waypointsPath.num_waypoints] = end_node[0:3]
            waypointsPath.airspeed[waypointsPath.num_waypoints] = end_node[4]
            waypointsPath.course[waypointsPath.num_waypoints] = end_node[3]
            waypointsPath.num_waypoints = 1
            return waypointsPath
        else:
            numPaths = 0
            while numPaths < 1: #?? Change to do full trees multiple times. Then pick best path. Need to calculate dubin cost though to compare.
                tree, flag = self.extendTree(tree, end_node, R_min, map, pd)
                numPaths = numPaths + flag


        # find path with minimum cost to end_node
        minPath = self.findMinimumPath(tree, end_node)
        return self.smoothPath(minPath, map, R_min)
        # return self.findMinimumPath(tree, end_node)

    def randomPoint(self, map):
        return np.random.uniform(low=0, high=map.city_width), np.random.uniform(low=0, high=map.city_width)

    def collision(self, startNode, endNode, map, radius):
        dubinPath = dubins_parameters()
        dubinPath.update(startNode[0:3], startNode.item(3), endNode[0:3], endNode.item(3), radius)
        if np.isnan(dubinPath.r1.item(1)):
            return True
        N, E, D = self.pointsAlongDubinsPath(dubinPath, self.pointsAlongPathSpacing)
        # spacing = map.city_width / map.num_city_blocks
        # N += (spacing * .5)
        # E += (spacing * .5)
        # N = np.remainder(N, spacing)
        # E = np.remainder(E, spacing)

        #Check for within circle of square obstacle
        for i in range(0,len(map.building_north)):
            norths = np.abs(N - map.building_north[i])
            withinNorth = (norths < (map.building_width/2+self.clearance))
            if np.any(withinNorth, axis=0):
                for j in range(0, len(map.building_east)):
                    easts = np.abs(E[withinNorth] - map.building_east[j])
                    withinEast = (easts < (map.building_width/2+self.clearance))
                    downSub = D[withinNorth]
                    if np.any(withinEast, axis=0):
                        if np.any(-downSub[withinEast] < (map.building_height[j,i]+self.clearance)):
                            return True
        return False



    def pointsAlongPath(self, start_node, end_node, Del):
        N = np.array([start_node.item(0)])
        E = np.array([start_node.item(1)])
        D = np.array([start_node.item(2)])

        q = end_node[0:3]-start_node[0:3]
        # q = np.array([endN.n - start_node.item(0), endN.e - start_node.item(1), endN.d - startN.d])
        L = np.linalg.norm(q)
        q = q / L

        w = np.array(start_node[0:3])
        for i in range(1, int(np.ceil(L / Del))):
            w += Del * q
            N = np.append(N, w.item(0))
            E = np.append(E, w.item(1))
            D = np.append(D, w.item(2))
        N = np.append(N, end_node.item(0))
        E = np.append(E, end_node.item(1))
        D = np.append(D, end_node.item(2))
        return N, E, D

    # def downAtNE(self, map, n, e):

    def extendTree(self, tree, endN, R_min, map, pd):
        successFlag = False
        while not successFlag:
            # Generate Random Point
            northP, eastP = self.randomPoint(map)

            # Find nearest leaf.
            distances = (northP-tree[:, 0])**2 + (eastP-tree[:, 1])**2
            minIndex = np.argmin(distances)  # could loop through a second time to try second best node?? This might help with smoother ascending and descending.

            chi = self.mod(np.arctan2((eastP - tree[minIndex, 1]), (northP - tree[minIndex, 0])))
            # Calculate the new node location
            # A new leaf only extends a maximum distance from a previous leaf
            L = min(np.sqrt((northP-tree[minIndex,0])**2 + (eastP-tree[minIndex,1])**2), self.segmentLength)
            downP = endN.item(2)
            tmp = np.array([northP, eastP, downP]) - np.array([tree[minIndex,0], tree[minIndex,1], tree[minIndex,2]])
            newPoint = np.array([tree[minIndex, 0], tree[minIndex, 1], tree[minIndex, 2]]) + L * (tmp / np.linalg.norm(tmp))
            newNode = np.array([[newPoint.item(0), newPoint.item(1), newPoint.item(2), chi, tree[minIndex, 3] + L, minIndex, 0.]])

            # Check for Collision
            if np.linalg.norm(tree[minIndex, 0:3] - newNode[0, 0:3]) > 3.*R_min and \
                not self.collision(tree[minIndex, :], newNode[0, :], map, R_min):
                successFlag = True
                tree = np.append(tree, newNode,axis=0)  # Append new node to the full tree
                # points = self.ax.plot([tree[minIndex, 0], newNode.item(0)], [tree[minIndex, 1], newNode.item(1)],
                #                       [-tree[minIndex, 2], -newNode.item(2)], color='r')

                # Check to see if the new node can connect to the end node
                dist = np.linalg.norm(newNode[0, 0:3]-endN[0:3])
                if np.size(tree, 0) == 35:
                    a = 2
                # chi = np.arctan2((endN.e - newNode.item(1)), (endN.n - newNode.item(0)))
                if np.linalg.norm(newNode[0, 0:3] - endN[0:3]) >= 3.*R_min \
                        and dist < 1.5*self.segmentLength and not self.collision(newNode[0,:], endN, map, R_min):
                    tree[np.size(tree, 0)-1, 6] = 1
                    return tree, 1  # Return the extended tree with the flag of a successful path to ending node
                else:
                    return tree, 0

    def findMinimumPath(self, tree, end_node):
        # Find the leaves that connect to the end node
        connectedNodes = []
        for i in range(0, np.size(tree, 0)):
            if tree[i, 6] == 1:
                connectedNodes.append(i)

        # Find the path with the shortest distance (could find a different heuristic for choosing which path to go with,
        # especially because we are going to shorten the path anyway??). Choose shortest after smoothing?? Or choose for
        # least turns.
        minIndex = np.argmin(tree[connectedNodes, 3])
        minIndex = connectedNodes[minIndex]
        waypoints = msg_waypoints()
        waypoints.ned[:,waypoints.num_waypoints] = end_node[0:3]
        waypoints.course[:,waypoints.num_waypoints] = end_node.item(3)
        waypoints.num_waypoints += 1
        waypoints.ned[:,waypoints.num_waypoints] = tree[minIndex,0:3]
        waypoints.course[:,waypoints.num_waypoints] = tree[minIndex,3]
        waypoints.num_waypoints += 1
        parentNode = int(tree[minIndex, 5])
        while parentNode > 0:
            waypoints.ned[:,waypoints.num_waypoints] = tree[parentNode, 0:3]
            waypoints.course[:,waypoints.num_waypoints] = tree[parentNode,3]
            waypoints.num_waypoints += 1
            parentNode = int(tree[parentNode, 5])
        waypoints.ned[:,waypoints.num_waypoints] = tree[parentNode, 0:3]
        waypoints.course[:, waypoints.num_waypoints] = tree[parentNode, 3]
        waypoints.num_waypoints += 1  # This adds the starting point
        waypoints2 = msg_waypoints()
        for i in range(0, waypoints.num_waypoints):
            waypoints2.ned[:,i] = waypoints.ned[:,waypoints.num_waypoints-i-1]
            waypoints2.course[:,i] = waypoints.course[:,waypoints.num_waypoints-i-1]
        waypoints.ned = waypoints2.ned
        waypoints.course = waypoints2.course
        return waypoints

    def smoothPath(self, path, map, R_min):
        waypoints = msg_waypoints()
        waypoints.ned[:, waypoints.num_waypoints] = path.ned[:, waypoints.num_waypoints]
        waypoints.course[:, waypoints.num_waypoints] = path.course[:,waypoints.num_waypoints]
        waypoints.num_waypoints += 1
        index = 1
        while index < path.num_waypoints - 1:
            # chi = np.arctan2((path.ned[1,index + 1] - waypoints.ned[1,waypoints.num_waypoints-1]),
            #                  (path.ned[0,index + 1] - waypoints.ned[0,waypoints.num_waypoints-1]))
            if self.collision(np.concatenate((waypoints.ned[:,waypoints.num_waypoints-1],waypoints.course[:,waypoints.num_waypoints-1]),axis=0), np.concatenate((path.ned[:,index + 1],path.course[:,index + 1]),axis=0), map, R_min) or \
                np.linalg.norm(waypoints.ned[:,waypoints.num_waypoints-1] - path.ned[:,index + 1]) <= 2.*R_min:
                # self.flyablePath(smoothedPath[len(smoothedPath) - 1], path[index + 1], prev_chi, chi):
                waypoints.ned[:,waypoints.num_waypoints] = path.ned[:, index]
                waypoints.course[:,waypoints.num_waypoints] = path.course[:,index]
                waypoints.airspeed[:,waypoints.num_waypoints] = path.airspeed[:,index]
                waypoints.num_waypoints += 1
            index += 1
        waypoints.ned[:,waypoints.num_waypoints] = path.ned[:,path.num_waypoints-1]
        waypoints.course[:, waypoints.num_waypoints] = path.course[:, path.num_waypoints - 1]
        waypoints.airspeed[:, waypoints.num_waypoints] = path.airspeed[:, path.num_waypoints - 1]
        waypoints.num_waypoints += 1
        # smoothedPath.append(path[len(path) - 1])
        # for i in range(0, len(smoothedPath) - 1):  # Could add other things to this cost function if wanted
        #     cost += np.sqrt(
        #         (smoothedPath[i].n - smoothedPath[i + 1].n) ** 2 + (smoothedPath[i].e - smoothedPath[i + 1].e) ** 2 + \
        #         (smoothedPath[i].d - smoothedPath[i + 1].d) ** 2)

        return waypoints

    def rotz(self, theta):
        R = np.array([[np.cos(theta), -np.sin(theta), 0],
                      [np.sin(theta), np.cos(theta), 0],
                      [0, 0, 1]])
        return R

    def pointsAlongDubinsPath(self, dubinspath, Del):
    # point along start circle
        th1 = self.mod(np.arctan2(dubinspath.p_s.item(1) - dubinspath.center_s.item(1), dubinspath.p_s.item(0) - dubinspath.center_s.item(0)))
        th2 = self.mod(np.arctan2(dubinspath.r1.item(1) - dubinspath.center_s.item(1), dubinspath.r1.item(0) - dubinspath.center_s.item(0)))
        if dubinspath.dir_s > 0:
            if th1 >= th2:
                th = np.concatenate((np.arange(th1,2*np.pi, Del), np.arange(0, th2, Del)), axis=0)
                # th = [th1:Del: 2 * pi, 0: Del:th2]
            else:
                th = np.arange(th1, th2, Del)
        else:
            if th1 <= th2:
                th = np.concatenate((np.arange(th1, 0,-Del), np.arange(2*np.pi, th2, -Del)), axis=0)
            else:
                th = np.arange(th1, th2, -Del)
        X = np.array([])
        Y = np.array([])
        Z = np.array([])
        for i in range(0, len(th)):
            X = np.append(X, dubinspath.center_s.item(0) + dubinspath.radius * np.cos(th[i]))
            Y = np.append(Y, dubinspath.center_s.item(1) + dubinspath.radius * np.sin(th[i]))
            Z = np.append(Z, -dubinspath.center_s.item(2))
            # X = [X, dubinspath.cs(1) + dubinspath.R * np.cos(th(i))];
            # Y = [Y, dubinspath.cs(2) + dubinspath.R * np.sin(th(i))];


        # points along straight line
        sig = 0
        while sig <= 1:
            X =np.append(X,(1 - sig) * dubinspath.r1.item(0) + sig * dubinspath.r2.item(0))
            # X = [X; (1 - sig) * dubinspath.w1(1) + sig * dubinspath.w2(1)];
            Y = np.append(Y, (1 - sig) * dubinspath.r1.item(1) + sig * dubinspath.r2.item(1))
            Z = np.append(Z, -(1 - sig) * dubinspath.r1.item(2) - sig * dubinspath.r2.item(2))
            # Y = [Y; (1 - sig) * dubinspath.w1(2) + sig * dubinspath.w2(2)];
            sig += Del

        # points along end circle
        th2 = self.mod(np.arctan2(dubinspath.p_e.item(1) - dubinspath.center_e.item(1), dubinspath.p_e.item(0) - dubinspath.center_e.item(0)))
        th1 = self.mod(np.arctan2(dubinspath.r2.item(1) - dubinspath.center_e.item(1), dubinspath.r2.item(0) - dubinspath.center_e.item(0)))
        if dubinspath.dir_e > 0:
            if th1 >= th2:
                th = np.concatenate((np.arange(th1, 2 * np.pi, Del),np.arange(0, th2, Del)) , axis=0)
                # th = [th1:Del: 2 * pi, 0: Del:th2];
            else:
                th = np.arange(th1, th2, Del)
        else:
            if th1 <= th2:
                # th = [th1:-Del: 0, 2 * pi: -Del:th2];
                th = np.concatenate((np.arange(th1, 0, -Del), np.arange(2 * np.pi, th2, -Del)), axis=0)
            else:
                th = np.arange(th1, th2, -Del)
        for i in range(0,len(th)):
            X = np.append(X, dubinspath.center_e.item(0) + dubinspath.radius * np.cos(th[i]))
            Y = np.append(Y, dubinspath.center_e.item(1) + dubinspath.radius * np.sin(th[i]))
            Z = np.append(Z, -dubinspath.center_e.item(2))
        return X, Y, -Z

    def mod(self, x):
        # make x between 0 and 2*pi
        while x < 0:
            x += 2*np.pi
        while x > 2*np.pi:
            x -= 2*np.pi
        return x
