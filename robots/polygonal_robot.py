# 3rd-party packages
import numpy as np
import copy

# local packages
from robots.robot import Robot
from factory.builder import Builder
from spaces.factory import activeSpaces


##
# @brief      This class describes a PolygonalRobot with a workspace shape that
# can be described as a set of polygonal vertices
#
class PolygonalRobot(Robot):

    ##
    # @brief      PolygonalRobot class constructor
    #
    # @param      robotType        The PolygonalRobot type string
    # @param      configData       Configuration dictionary for the
    #                              PolygonalRobot
    # @param      workspace        The Workspace object the PolygonalRobot
    #                              operates in
    # @param      shouldSavePlots  Boolean controlling whether or not the plt
    #                              objs can be saved to the baseSaveName dir
    # @param      baseSaveFName    The base directory file name for output plot
    #
    # @return     initialized PolygonalRobot object
    #
    def __init__(self, robotType, configData, workspace,
                 shouldSavePlots, baseSaveFName):

        # calling the superclass contructor to inherit its properties
        Robot.__init__(self,
                       robotType=robotType,
                       configData=configData,
                       workspace=workspace,
                       shouldSavePlots=shouldSavePlots,
                       baseSaveFName=baseSaveFName)

        (relativeShapeVerts, robotOriginIdx) = self.setRobotShape(configData)

        self.workspaceVertCoords = []

        # this is how the robot's vertices are the robot's verices in its
        # local coordinate system
        self.relativeShapeVerts = relativeShapeVerts

        # need to store which of the robot's vertices is its origin
        self.robotOriginIdx = robotOriginIdx

        # have the factory get make the PolygonalRobot's C-space
        self.cSpace = activeSpaces.get(robotSpaceType='POLYGONALROBOTCSPACE',
                                       robot=self,
                                       shouldSavePlots=shouldSavePlots,
                                       baseSaveFName=baseSaveFName)

    ##
    # @brief      Extracts and formats the PolygonalRobot shape data from the
    #             configData
    #
    # @param      configData  Configuration dictionary for the PolygonalRobot
    #
    # @return     (the coordinates of the robot's verts in workspace,
    #              the coordinates of the robot's verts rel to its origin,
    #              the index of the robot's origin vertex in the list of verts)
    #
    def setRobotShape(self, configData):

        relativeShapeVerts = np.array(configData['robotShape'],
                                      dtype='float64')
        robotOriginIdx = configData['robotOriginVertIdx']
        robotOrigin = np.array(configData['robotOriginLocInWokspace'],
                               dtype='float64')
        robotOrigin.shape = (2, 1)

        return (relativeShapeVerts, robotOriginIdx)

    ##
    # @brief      Gets the workspace vertices of the robot given its new origin
    #             and a rotation by theta
    #
    #
    #             as the shape of the PolygonalRobot in workspace is given as
    #             relative coordinates from the PolygonalRobot workspace
    #             origin, we need to offset the relative vertex coordinates by
    #             the origin to get the actual position of the PolygonalRobot's
    #             verts in the workspace
    #
    # @param      relativeShapeVerts  the coordinates of the robot's verts
    #                                 relative to its origin
    # @param      robotOriginIdx      the index of the robot's origin vertex in
    #                                 the list of verts
    # @param      newOriginPos        The robot's new origin position in
    #                                 Workspace coordinates as np.array
    # @param      theta               The angle to rotate the robot about its
    #                                 origin by [rad]
    #
    # @return     The coordinates of the robot's vertices in the workspace
    #
    def getNewWorkspaceVertices(self, relativeShapeVerts, robotOriginIdx,
                                newOriginPos, theta):

        # translate the robot's vertices' coordinates to be about the new
        # origin
        currRobotVertCoords = relativeShapeVerts + newOriginPos.T

        # rotate the robot's vertices' coordinates about the origin by theta
        newRobotVertCoords = self.rotateRobot(theta, currRobotVertCoords,
                                              robotOriginIdx)

        return newRobotVertCoords

    ##
    # @brief      Puts the robot into the newState and updates appropriate data
    #
    # @param      newState  The new state for the PolygonalRobot
    #
    def updateRobotState(self, newState):

        self.currentState = newState
        self.stateHistory.append(copy.deepcopy(newState))
        self.distTraveled += self.distToTarget(newState)

        # need to compute the robot's pose after the change in state
        newOriginPos = newState[0:2, :]
        theta = np.asscalar(newState[2])

        newVertCoors = self.getNewWorkspaceVertices(self.relativeShapeVerts,
                                                    self.robotOriginIdx,
                                                    newOriginPos,
                                                    theta)
        self.workspaceVertCoords.append(newVertCoors)

    ##
    # @brief      rotates all of the robot's workspace coordinates by theta
    #
    # @param      theta            The body frame orientation state vector
    #                              component [rad]
    # @param      robotVertCoords  The robot's vertices' coordinates in
    #                              workspace
    # @param      robotOriginIdx   the index of the robot's origin vertex in
    #                              the list of verts
    #
    def rotateRobot(self, theta, robotVertCoords, robotOriginIdx):

        # define the rotate matrix about the origin
        r = np.array(((np.cos(theta), -np.sin(theta)),
                      (np.sin(theta), np.cos(theta))))

        # extract the location of the robot's origin vertex
        origin = robotVertCoords[robotOriginIdx:robotOriginIdx + 1, :].T

        # transpose the collection of vertex coordinates so it a matrix of
        # column vectors of vertex coordinates, then subtract out the point of
        # rotation
        verts = robotVertCoords.T - origin

        # rotate all of the coordinates by theta, centered about the global
        # workspace origin
        vertsRot = np.matmul(r, verts)

        # now add the origin robot origin vert coords back to the result to get
        # the coordinates back in the right place in the global coordinate
        # system, then transpose the result to match the original format of the
        # list of vertices
        vertsFinal = np.transpose(vertsRot + origin)

        return vertsFinal

    ##
    # @brief      Plots the robot in the workspace
    #
    # @param      ax    the matplotlib.axes object to plot the PolygonalRobot's
    #                   body in its Workspace
    #
    # @return     plots the PolygonalRobot's body on ax
    #
    def plotBodyInWorkspace(self, ax):

        for vertCoords in self.workspaceVertCoords:
            ax.fill(vertCoords[:, 0], vertCoords[:, 1],
                    facecolor='lightsalmon',
                    edgecolor='orangered',
                    linewidth=3)

    ##
    # @brief      Linearly evolves the robot's config state from its start to
    #             goal state
    #
    #             For the PolygonalRobot, we are just going to linearly evolve
    #             all config states from startState to goalState (without a
    #             planner) as we are just testing c-space construction and
    #             visualization
    #
    # @param      startState  The start state in C-Space
    # @param      endState    The end state in C-Space
    #
    def linearlyEvolveState(self, startState, endState):

        N = 100

        # fuck python
        currState = copy.deepcopy(startState)
        x0, y0, theta0 = startState
        xG, yG, thetaG = endState
        self.updateRobotState(currState)

        # calculate linear differentials
        dx = float((xG - x0) / N)
        dy = float((yG - y0) / N)
        d_theta = float((thetaG - theta0) / N)

        for ii in range(0, N):

            currState[0] += dx
            currState[1] += dy
            currState[2] += d_theta

            self.updateRobotState(currState)

    ##
    # @brief      Function for PolygonalRobot instance to "run" itself
    #
    #             For the PolygonalRobot, we are just going to linearly evolve
    #             all config states from startState to goalState (without a
    #             planner) as we are just testing c-space construction and
    #             visualization
    #
    # @param      planner    The planner object containing the motion planning
    #                        algorithm to be used on the robot
    # @param      plotTitle   The plot title string
    #
    # @return     runs robot, then plots results of running the robot
    #
    def runAndPlot(self, planner, plotTitle):

        self.workspace.plot(robot=self,
                            startState=self.startState,
                            goalState=self.goalState,
                            plotTitle=plotTitle + 'workspace')

        self.cSpace.plot(robot=self,
                         startState=self.startState,
                         goalState=self.goalState,
                         plotTitle=plotTitle + 'cSpace')


##
# @brief      Implements the generic builder class for the PolygonalRobot
#
class PolygonalRobotBuilder(Builder):

    ##
    # @brief need to call the super class constructor to gain its properties
    #
    def __init__(self):
        Builder.__init__(self)

    ##
    # @brief      Implements the smart constructor for PolygonalRobot
    #
    #             Only reads the config data once, otherwise just returns the
    #             built object
    #
    # @param      robotType        The PolygonalRobot type string
    # @param      configFileName   The configuration file name
    # @param      workspace        The Workspace object the PolygonalRobot
    #                              operates in
    # @param      shouldSavePlots  Boolean controlling whether or not the plt
    #                              objs can be saved to the baseSaveName dir
    # @param      baseSaveFName    The base directory file name for output plot
    #
    # @return     instance of an initialized PolygonalRobot object
    #
    def __call__(self, robotType, configFileName, workspace, shouldSavePlots,
                 baseSaveFName):

        configNameHasChanged = (self._configName != configFileName)
        noInstanceLoadedYet = (self._instance is None)

        if noInstanceLoadedYet or configNameHasChanged:

            configData = self.loadConfigData(configFileName)
            self._instance = PolygonalRobot(robotType=robotType,
                                            configData=configData,
                                            workspace=workspace,
                                            shouldSavePlots=shouldSavePlots,
                                            baseSaveFName=baseSaveFName)

        return self._instance
