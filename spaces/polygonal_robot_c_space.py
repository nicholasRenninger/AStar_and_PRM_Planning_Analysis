# 3rd-party packages
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import art3d
import matplotlib.colors as colors
import pylab as pl
import scipy as sp
import scipy.spatial
import numpy as np
from collections import deque
import copy

# local packages
from factory.builder import Builder
from spaces.robot_space import RobotSpace
from util.plots import savePlot


##
# @brief      Class implementing a PolygonalRobot's configuration space object
#
class PolygonalRobotCSpace(RobotSpace):

    ##
    # @brief      PolygonalRobotCSpace class constructor
    #
    # @param      robot            The PolygonalRobot instance
    # @param      shouldSavePlots  Boolean controlling whether or not the plt
    #                              objs can be saved to the baseSaveName dir
    # @param      baseSaveFName    The base directory file name for output plot
    #
    # @return     initialized PolygonalRobotCSpace object
    #
    def __init__(self, robot, shouldSavePlots, baseSaveFName):

        # get inherited properties from superclass implementation
        RobotSpace.__init__(self, shouldSavePlots, baseSaveFName)

        self.workspace = robot.workspace
        self.robot = robot

        # ugh, compute c-space obstacles
        self.obstacles = self.getObstacles(robot, self.workspace)

        print('Initial Robot config coordinates:')
        print(robot.startState)
        print('Final Robot config coordinates:')
        print(robot.currentState)

        print('Initial CSpace obstacle Coordinates:')
        print(self.obstacles[0])
        print('Last CSpace obstacle Coordinates:')
        print(self.obstacles[-1])

    ##
    # @brief      Calculates PolygonalRobotCSpace obstacles from the robot type
    #             and the obstacles in the workspace
    #
    # @param      robot      The PolygonalRobot
    # @param      workspace  The Workspace object associated with the robot
    #
    # @return     a list of PolygonalRobotCSpace obstacle vertex coordinates
    #             for each obstacle
    #
    def getObstacles(self, robot, workspace):

        # start by envolving the state through its desired path in CSpace
        robot.linearlyEvolveState(robot.startState, robot.goalState)

        # need to sort the lists of vertices so that they're both CCW and such
        # that the smallest y value is the first array element
        allSortedRobotVerts = []
        for robotVerts in robot.workspaceVertCoords:
            allSortedRobotVerts.append(self.sortVertsCCW_Ymin(robotVerts))

        allSortedObstacleVerts = []
        for obstacle in workspace.obstacles:
            allSortedObstacleVerts.append(self.sortVertsCCW_Ymin(obstacle))

        # compute the angle that each surface normal makes from each of the
        # robot's facets in the workspace coordinate system
        robotFacetAngles = []
        for robotVerts in allSortedRobotVerts:
            currRobAngles = self.computeFacetAngles(robotVerts)
            robotFacetAngles.append(currRobAngles)

        # compute the angle that each surface normal makes from each of the
        # obstacle's facets in the workspace coordinate system
        obstaclesFacetAngles = []
        for obstacle in allSortedObstacleVerts:
            currObstAngles = self.computeFacetAngles(obstacle)
            obstaclesFacetAngles.append(currObstAngles)

        # compute the minkowski sum for each obstacle / robot config
        cSpaceObstacles = []
        for (oVert, oAngle) in zip(allSortedObstacleVerts,
                                   obstaclesFacetAngles):
            for (rVert, rAngle, rState) in zip(allSortedRobotVerts,
                                               robotFacetAngles,
                                               robot.stateHistory):
                oVert = np.array(oVert)
                newCSpaceObstacle2D = self.computeMinkowskiSum(oAngle, oVert,
                                                               rAngle, rVert)

                # now need to add the height of each obstacle in c-Space, i.e.
                # the theta value of the robot's state for each obstacle
                nObstVerts = len(newCSpaceObstacle2D)
                newCSpaceObstacle = np.zeros((nObstVerts, 3), dtype='float64')
                theta = rState[2]

                for i in range(0, nObstVerts):
                    newCSpaceObstacle[i, 0:2] = newCSpaceObstacle2D[i][:]
                    newCSpaceObstacle[i, 2] = theta

                cSpaceObstacles.append(newCSpaceObstacle)

        return cSpaceObstacles

    ##
    # @brief      sorts the list of convex vertices such that they are CCW and
    #             the coordinate with the smallest y-coordinate is first
    #
    # @param      verts  The vertices to sort
    #
    # @return     the vert data structure with the first element being the one
    #             with smallest y value, and the vertices in CCW order from the
    #             first vertex
    #
    def sortVertsCCW_Ymin(self, verts):

        hull = sp.spatial.ConvexHull(verts)
        convexVerts = verts[hull.vertices]

        # need to keep CCW order while making the first element of the vertex
        # list the one with the smallest y value. Do this by putting vertices
        # with larger y value in a queue until you find the min vertex. use the
        # remainder of the vertex list as the new queue, and append the front
        # of the queue into the new list to get a vertex list still in CCW
        # order
        minY = np.min(convexVerts[:, 1])
        nVerts = len(convexVerts[:, 1])

        # build queue of larger y elements
        i = 0
        outOfOrderVertQueue = deque()
        currVertY = convexVerts[i, 1]
        while (i < nVerts) and (currVertY > minY):

            outOfOrderVertQueue.append(convexVerts[i, :])
            i += 1
            currVertY = convexVerts[i, 1]

        # create new, partial CCW vertex list with minY vertex as first elem
        nInOrderElems = nVerts - i
        ySortedConvexVerts = np.zeros(convexVerts.shape)
        ySortedConvexVerts[0:nInOrderElems] = copy.deepcopy(convexVerts[i:])

        endOfPartialList = nInOrderElems
        # now add the rest of the vertices that used to appear before the minY
        # elem in CCW order to the final output
        for (idx, vert) in enumerate(outOfOrderVertQueue):
            ySortedConvexVerts[endOfPartialList + idx, :] = vert

        return ySortedConvexVerts

    ##
    # @brief      Calculates the angle of each surface normal from the facet
    #             defined by neighbor vertices of a polygon
    #
    # @param      vertices  A list of vertices to copute surface normal angles
    #                       for
    #
    # @return     A dictionary of angles indexed by a key of the two vertices
    #             used to compute the angle value
    #
    def computeFacetAngles(self, vertices):

        # store as a dict as it allows for sparse lookup of angles
        facetAngles = {}
        nVerts = len(vertices)
        for i in range(0, nVerts):

            # need the indices to wrap around to the first vertex from the last
            idx2 = (i + 1) % nVerts
            idx1 = i

            # compute the slope components of the object's surface  in
            # workspace
            slopeX = vertices[idx2][0] - vertices[idx1][0]
            slopeY = vertices[idx2][1] - vertices[idx1][1]

            # compute the slope compnents of the normal to the surface
            newSlopeY = -slopeX
            newSlopeX = slopeY

            # compute the angle of the surface normal in the global workspace
            # coord. system
            angle = np.arctan2(newSlopeY, newSlopeX)

            # store the angles in for use in the computeMinkowskiSum algorithm
            facetAngles[(i, i + 1)] = angle

        return facetAngles

    ##
    # @brief      Calculates the minkowski sum of the obstacle and robot
    #             vertices
    #
    # @warning    Assumes that the robot and obstacles have vertices ordered in
    #             CCW order, with the vertex with the minimum y-coordinate in
    #             each set of coordinates being the first vertex in the list of
    #             CCW vertices
    #
    # @param      obstacleAngles  The obstacle's surface normal angles
    #                             dictionary
    # @param      obstacleVerts   The obstacle's workspace vertices
    # @param      robotAngles     The robot's surface normal angles dictionary
    # @param      robotVerts      The robot's workspace vertices
    #
    # @return     The minkowski sum.
    #
    def computeMinkowskiSum(self, obstacleAngles, obstacleVerts,
                            robotAngles, robotVerts):

        i = 0
        j = 0

        n = len(obstacleAngles)
        m = len(robotAngles)

        minkowskiSum = []

        while (i <= n - 1) and (j <= m - 1):

            newCSpaceObstVert = obstacleVerts[i] + robotVerts[j]
            minkowskiSum.append(newCSpaceObstVert)

            currRobotAngle = robotAngles[(j, j + 1)]
            currObstacleAngle = obstacleAngles[(i, i + 1)]

            if currObstacleAngle < currRobotAngle:
                i += 1

            elif currObstacleAngle > currRobotAngle:
                j += 1

            else:
                i += 1
                j += 1

        return minkowskiSum

    ##
    # @brief      Plots all CSpace objects and saves to self.baseSaveFName
    #
    #             plots obstacles, the robot's path, the start location, and
    #             the goal location
    #
    # @param      robot           A list with spatial coordinates of the
    #                             robot's path in the CSpace coordinate system
    # @param      startState      A list with the robot's start coordinates in
    #                             the CSpace coordinate system
    # @param      goalState       A list with the robot's goal state
    #                             coordinates in the CSpace coordinate system
    # @param      plotConfigData  The plot configuration data dictionary:
    #                             - plotTitle  The plot title string
    #                             - xlabel     xlabel string
    #                             - ylabel     ylabel string
    #
    # @return     a plot of the CSpace in the self.baseSaveFName directory
    #
    def plot(self, robot, startState, goalState, plotConfigData):

        # unpack dictionary
        plotTitle = plotConfigData['plotTitle']
        xlabel = plotConfigData['xlabel']
        ylabel = plotConfigData['ylabel']

        fig = plt.figure()
        ax = fig.gca(projection='3d')

        # plot the robot's trajectory in Cspace
        robotPath = robot.stateHistory
        x = [state[0][0] for state in robotPath]
        y = [state[1][0] for state in robotPath]
        z = [state[2][0] for state in robotPath]
        ax.plot(x, y, z, color='blue', linestyle='solid',
                linewidth=4, markersize=16)

        # plot all of the CSpace obstacles
        for obstacle in self.obstacles:

            obstaclePolygon = art3d.Poly3DCollection([obstacle], alpha=0.8)
            face_color = [0.5, 0.5, 1]
            obstaclePolygon.set_color(colors.rgb2hex(face_color))
            obstaclePolygon.set_edgecolor('k')
            ax.add_collection3d(obstaclePolygon)

        pl.title(plotTitle)
        pl.xlabel(xlabel)
        pl.ylabel(ylabel)

        ax.set_xlim3d(2, 8)
        ax.set_ylim3d(2, 8)
        ax.set_zlim3d(0, 7)

        savePlot(fig=fig, shouldSavePlots=self.shouldSavePlots,
                 baseSaveFName=self.baseSaveFName, plotTitle=plotTitle)

        return None


##
# @brief      Implements the generic builder class for the PolygonalRobotCSpace
#
class PolygonalRobotCSpaceBuilder(Builder):

    # need to call the super class constructor to gain its properties
    #
    def __init__(self):
        Builder.__init__(self)
        self.robot = None
        self.workspace = None

    ##
    # @brief      Implements the smart constructor for PolygonalRobotCSpace
    #
    #             Only reads the config data once, otherwise just returns the
    #             built object
    #
    # @param      robot            The PolygonalRobotCSpace type string
    # @param      shouldSavePlots  Boolean controlling whether or not the plt
    #                              objs can be saved to the baseSaveName dir
    # @param      baseSaveFName    The base directory file name for output plot
    #
    # @return     instance of an initialized PolygonalRobotCSpace object
    #
    def __call__(self, robot, shouldSavePlots, baseSaveFName):

        robotHasChanged = (self.robot != robot)
        workspaceHasChanged = (self.workspace != robot.workspace)
        noInstanceLoadedYet = (self._instance is None)

        if noInstanceLoadedYet or robotHasChanged or workspaceHasChanged:

            self._instance = \
                PolygonalRobotCSpace(robot=robot,
                                     shouldSavePlots=shouldSavePlots,
                                     baseSaveFName=baseSaveFName)

        return self._instance
