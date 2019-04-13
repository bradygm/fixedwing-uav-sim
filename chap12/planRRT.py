import numpy as np
from message_types.msg_waypoints import msg_waypoints


#This could be much better if it picked the best path after all were smoothed.
class planRRT():
    def __init__(self, map):
        self.segmentLength = 300 # standard length of path segments
        self.pointsAlongPathSpacing = 5.
        self.clearance = 30
        # np.random.seed(1112)  # For Debugging

    def planPath(self, wpp_start, wpp_end, map):

        # desired down position is down position of end node
        pd = wpp_end.item(2)

        # specify start and end nodes from wpp_start and wpp_end
        # format: N, E, D, cost, parentIndex, connectsToGoalFlag,
        start_node = np.array([wpp_start.item(0), wpp_start.item(1), pd, 0, -1, 0])
        end_node = np.array([wpp_end.item(0), wpp_end.item(1), pd, 0, 0, 0])

        # establish tree starting with the start node
        tree = np.array([start_node])

        # check to see if start_node connects directly to end_node
        if ((np.linalg.norm(start_node[0:3] - end_node[0:3]) < self.segmentLength ) and not self.collision(start_node, end_node, map)):
            waypointsPath = msg_waypoints()
            waypointsPath.ned[:,0:2] = np.append([start_node[0:3]], [end_node[0:3]],axis=0).T
            # waypointsPath.airspeed[0:2] = np.append(start_node[3],end_node[3],axis=0)
            # np.append(tree, newNode, axis=0)
            waypointsPath.num_waypoints = 2
            return waypointsPath
        else:
            numPaths = 0
            while numPaths < 3:
                tree, flag = self.extendTree(tree, end_node, map, pd)
                numPaths = numPaths + flag


        # find path with minimum cost to end_node
        path = self.findMinimumPath(tree, end_node)
        return self.smoothPath(path, map)

    def randomPoint(self, map):
        return np.random.uniform(low=0, high=map.city_width), np.random.uniform(low=0, high=map.city_width)

    def collision(self, startNode, endNode, map):
        N, E, D = self.pointsAlongPath(startNode, endNode, self.pointsAlongPathSpacing)
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

    def extendTree(self, tree, endN, map, pd):
        successFlag = False
        while not successFlag:
            # Generate Random Point
            northP, eastP = self.randomPoint(map)

            # Find nearest leaf.
            distances = (northP-tree[:, 0])**2 + (eastP-tree[:, 1])**2
            minIndex = np.argmin(distances)  # could loop through a second time to try second best node?? This might help with smoother ascending and descending.

            # Calculate the new node location
            # A new leaf only extends a maximum distance from a previous leaf
            L = min(np.sqrt((northP-tree[minIndex,0])**2 + (eastP-tree[minIndex,1])**2), self.segmentLength)
            downP = endN.item(2)
            tmp = np.array([northP, eastP, downP]) - np.array([tree[minIndex,0], tree[minIndex,1], tree[minIndex,2]])
            newPoint = np.array([tree[minIndex, 0], tree[minIndex, 1], tree[minIndex, 2]]) + L * (tmp / np.linalg.norm(tmp))
            newNode = np.array([[newPoint.item(0), newPoint.item(1), newPoint.item(2), tree[minIndex, 3] + L, minIndex, 0.]])

            # Check for Collision
            if not self.collision(tree[minIndex,:], newNode[0,:], map):
                successFlag = True
                tree = np.append(tree, newNode,axis=0)  # Append new node to the full tree
                # points = self.ax.plot([tree[minIndex, 0], newNode.item(0)], [tree[minIndex, 1], newNode.item(1)],
                #                       [-tree[minIndex, 2], -newNode.item(2)], color='r')

                # Check to see if the new node can connect to the end node
                dist = np.linalg.norm(newNode[0, 0:3]-endN[0:3])
                if np.size(tree, 0) == 35:
                    a = 2
                # chi = np.arctan2((endN.e - newNode.item(1)), (endN.n - newNode.item(0)))
                if dist < self.segmentLength and not self.collision(newNode[0,:], endN, map):
                    tree[np.size(tree, 0)-1, 5] = 1
                    return tree, 1  # Return the extended tree with the flag of a successful path to ending node
                else:
                    return tree, 0

    def findMinimumPath(self, tree, end_node):
        # Find the leaves that connect to the end node
        connectedNodes = []
        for i in range(0, np.size(tree, 0)):
            if tree[i, 5] == 1:
                connectedNodes.append(i)

        # Find the path with the shortest distance (could find a different heuristic for choosing which path to go with,
        # especially because we are going to shorten the path anyway??). Choose shortest after smoothing?? Or choose for
        # least turns.
        minIndex = np.argmin(tree[connectedNodes, 3])
        minIndex = connectedNodes[minIndex]
        waypoints = msg_waypoints()
        waypoints.ned[:,waypoints.num_waypoints] = end_node[0:3]
        waypoints.num_waypoints += 1
        waypoints.ned[:,waypoints.num_waypoints] = tree[minIndex,0:3]
        waypoints.num_waypoints += 1
        parentNode = int(tree[minIndex, 4])
        while parentNode > 0:
            waypoints.ned[:,waypoints.num_waypoints] = tree[parentNode, 0:3]
            waypoints.num_waypoints += 1
            parentNode = int(tree[parentNode, 4])
        waypoints.ned[:,waypoints.num_waypoints] = tree[parentNode, 0:3]
        waypoints.num_waypoints += 1  # This adds the starting point
        waypoints2 = msg_waypoints()
        for i in range(0, waypoints.num_waypoints):
            waypoints2.ned[:,i] = waypoints.ned[:,waypoints.num_waypoints-i-1]
        waypoints.ned = waypoints2.ned
        return waypoints

    def smoothPath(self, path, map):
        waypoints = msg_waypoints()
        waypoints.ned[:, waypoints.num_waypoints] = path.ned[:, waypoints.num_waypoints]
        waypoints.num_waypoints += 1
        index = 1
        while index < path.num_waypoints - 1:
            if self.collision(waypoints.ned[:,waypoints.num_waypoints-1], path.ned[:,index + 1], map):
                # self.flyablePath(smoothedPath[len(smoothedPath) - 1], path[index + 1], prev_chi, chi):
                waypoints.ned[:,waypoints.num_waypoints] = path.ned[:, index]
                waypoints.num_waypoints += 1
            index += 1
        waypoints.ned[:,waypoints.num_waypoints] = path.ned[:,path.num_waypoints-1]
        waypoints.num_waypoints += 1
        waypoints.airspeed = path.airspeed
        # smoothedPath.append(path[len(path) - 1])
        # for i in range(0, len(smoothedPath) - 1):  # Could add other things to this cost function if wanted
        #     cost += np.sqrt(
        #         (smoothedPath[i].n - smoothedPath[i + 1].n) ** 2 + (smoothedPath[i].e - smoothedPath[i + 1].e) ** 2 + \
        #         (smoothedPath[i].d - smoothedPath[i + 1].d) ** 2)

        return waypoints

