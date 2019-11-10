# 3rd-party packages
import copy
from shapely.geometry import Polygon
import numpy as np
import math

# local packages
from robots.robot import Robot
from factory.builder import Builder
from spaces.factory import activeSpaces
from robots.link import Link


##
# @brief      This class describes a 2-link manipulator Robot subclass that has
#             one dimensional linksin the workspace
#
class ManipulatorRobot(Robot):

    ##
    # @brief      ManipulatorRobot class constructor
    #
    # @param      robotType        The ManipulatorRobot type string
    # @param      configData       Configuration dictionary for the
    #                              ManipulatorRobot
    # @param      workspace        The Workspace object the ManipulatorRobot
    #                              operates in
    # @param      shouldSavePlots  Boolean controlling whether or not the plt
    #                              objs can be saved to the baseSaveName dir
    # @param      baseSaveFName    The base directory file name for output plot
    #
    # @return     initialized ManipulatorRobot object
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

        # configure manipulator's links
        linkLengths = configData['linkLengths']
        jointOffset = configData['jointOffset']
        links = []
        for linkLength in linkLengths:
            link = Link(origin=[0, 0], linkLength=linkLength,
                        jointOffset=jointOffset)
            links.append(link)

        self.links = links
        self.numLinks = len(links)

        # need to save history of link updates for plotting motion of robot
        self.linkHistory = []

        # tolerancing for inverse kinematics
        self.IK_TOL = 1e-6

        # need to compute its start / goal cspace coordinates given the
        # starting workspace state
        self.cStateHistory = []
        (self.startCState,
         self.currentState,
         self.links) = self.setEffectorToDesired(self.startState)

        (self.goalCState, _, _) = self.setEffectorToDesired(self.goalState)

        # now "state" is the workspace end effector position after inverse
        # kinematics
        self.startState = self.currentState

        # now, call the state update routine to make sure everything is set
        # correctly after initialization
        self.updateRobotState(self.startCState)

        linearDiscretizationDensity = configData['linearDiscretizationDensity']

        # have the factory make the ManipulatorRobot's C-space
        self.cSpace = activeSpaces.get(robotSpaceType='MANIPULATORROBOTCSPACE',
                                       robot=self,
                                       N=linearDiscretizationDensity,
                                       shouldSavePlots=shouldSavePlots,
                                       baseSaveFName=baseSaveFName)

    ##
    # @brief      Sets the robot's end effector to the desired workspace state
    #
    # @param      desEff  The desired workspace state
    #
    # @return     (list of joint angles, coordinates of the end effector,
    #              the moved link objects)
    #
    def setEffectorToDesired(self, desEff):

        print('desired end effector pos:', desEff)

        (links, jointAngles,
         endEffLoc) = self.inverseKinematics(self.links, desEff)

        print('joint angles: ', jointAngles)
        print('effector at: ', endEffLoc)

        return jointAngles, endEffLoc, links

    ##
    # @brief      Computes the joint angles needed to move the manipulator to a
    #             certain state in the workspace (end effector location)
    #
    # @param      links                   The links objects of the manipulator
    # @param      desEndEffectorLocation  The desired end effector location in
    #                                     workspace
    #
    # @return     (the links of the robot moved to the correct location in
    #             workspace, the joint angles of the robot that get the end
    #             effector to the desired workspace location)
    #
    def inverseKinematics(self, links, desEndEffectorLocation):

        x = desEndEffectorLocation[0][0]
        y = desEndEffectorLocation[1][0]

        a1 = links[0].linkLength - 2 * links[0].jointOffset
        a2 = links[1].linkLength - 2 * links[1].jointOffset

        cosTheta2 = (1 / (2 * a1 * a2)) * ((x ** 2 + y ** 2) -
                                           (a1 ** 2 + a2 ** 2))

        theta2 = math.acos(cosTheta2)

        theta_1_1 = math.atan2(y, x)
        theta_1_2 = math.atan2(a2 * math.sin(theta2),
                               a1 + a2 * math.cos(theta2))
        theta1 = theta_1_1 - theta_1_2

        desJA = [theta1, theta2]
        (links, endEffLoc) = self.forwardKinematics(links, desJA)

        return (links, desJA, endEffLoc)

    ##
    # @brief      moves the robot's links to the desired joint angles
    #
    # @param      linksList       The list of links objects
    # @param      desJointAngles  The list of joint angles for each link
    #
    # @return     (links updated with rotated global coords, the end effector
    #             location)
    #
    def forwardKinematics(self, linksList, desJointAngles):

        links = copy.deepcopy(linksList)

        i = 0
        a = np.zeros(self.numLinks + 1)
        i += 1
        T = np.zeros((3, 3, len(links)))
        A = np.array([[0, 0, 1]]).T

        # compute link origins in global coordinates
        angleSum = 0
        for (link, jointAngle) in zip(links, desJointAngles):

            angleSum += jointAngle
            c_i = math.cos(jointAngle)
            s_i = math.sin(jointAngle)
            a[i] = link.linkLength - 2 * link.jointOffset

            T[:, :, i - 1] = np.array([[c_i, -s_i, a[i - 1]],
                                       [s_i, c_i, 0],
                                       [0, 0, 1]])
            if i == 1:
                prevProd = T[:, :, i - 1]
            else:
                prevProd = np.matmul(prevProd, T[:, :, i - 1])

            linkOrigin = np.matmul(prevProd, A)
            link.setEdgeVertices(prevProd)
            link.setAngle(angleSum)
            link.setOrigin(linkOrigin)

            i += 1

        # now need to compute the end effector location
        T_i = np.array([[1, 0, a[i - 1]],
                        [0, 1, 0],
                        [0, 0, 1]])
        T = np.matmul(prevProd, T_i)

        endEffectorLocationXYZ = np.matmul(T, A)
        endEffectorLocation = np.array([endEffectorLocationXYZ[0],
                                        endEffectorLocationXYZ[1]])

        return (links, endEffectorLocation)

    ##
    # @brief      Returns whether the given Shapely object collides with any
    #             workspace obstacles
    #
    # @param      shapelyObject  The shapely object to check for collision (not
    #                            used here)
    # @param      gridIndices    The grid indices to check for collion
    #
    # @return     True if an obstacle was computed to be at the gridIndices
    #
    def checkCollision(self, shapelyObject, gridIndices):

        collided = (self.cSpace.obstacles[gridIndices] == 1)

        return collided

    ##
    # @brief      Returns whether the link Polygon objects collide with any
    #             workspace obstacles
    #
    # @param      linkPolygons  The list of shapely objects corresponding to
    #                           the links to check for collision
    #
    # @return     True if the links collided with any obstacle, False otherwise
    #
    def checkLinkCollision(self, linkPolygons):

        obstacles = self.workspace.polygonObstacles

        for obstacle in obstacles:
            for linkPolygon in linkPolygons:

                if obstacle.intersects(linkPolygon):
                    return True

        return False

    ##
    # @brief      Returns whether the CSpace state collides with any workspace
    #             obstacles
    #
    # @param      cState  The configuration space state to check for collison
    #                     at
    #
    # @return     True if the CSpace state collides with any obstacle, False
    #             otherwise
    #
    def checkCollisionWithState(self, cState):

        # move the arm to the config state
        links, _ = self.forwardKinematics(self.links, desJointAngles=cState)

        # turn the links into polygons for checking intersection with obstacles
        linkPolygons = []
        for link in links:

            linkVerts = link.getEdgeVertices()
            linkPolygons.append(Polygon(linkVerts))

        # check all links for collision with all workspace obstacles at cState
        return self.checkLinkCollision(linkPolygons)

    ##
    # @brief      Puts the robot into a new cspace state and updates both the
    #             Cspace and workspace (end effector) state histories
    #
    # @param      newCState  The new cspace state for the ManipulatorRobot
    #
    def updateRobotState(self, newCState):

        # move the robot to the new cstate with forward kinematics
        (links,
         endEffectorLocation) = self.forwardKinematics(self.links,
                                                       newCState)
        self.links = copy.deepcopy(links)
        # need to keep track of how the link geometry changes with time
        self.linkHistory.append(copy.deepcopy(links))

        # need to track the end effector state history
        self.stateHistory.append(copy.deepcopy(endEffectorLocation))
        self.distTraveled += self.distToTarget(self.currentState,
                                               endEffectorLocation)
        self.endEffectorLocation = endEffectorLocation

        # also need to keep track of the configuration state over time
        self.currentState = newCState
        self.cStateHistory.append(copy.deepcopy(newCState))

    ##
    # @brief      Plots the snapshots of the robot's body's path in the
    #             workspace
    #
    # @param      ax    the matplotlib.axes object to plot the
    #                   ManipulatorRobot's body's path in its Workspace
    #
    # @return     plots the ManipulatorRobot's body's path on ax
    #
    def plotBodyInWorkspace(self, ax, fig):

        # plot only nth link snapshots to make the plot easier to read
        nLinksToPlot = 30

        # plot every nth link to make the plot easier to read
        numLinks = len(self.linkHistory)

        # determine how often to plot links i.e. plot every Nth link
        N = np.floor(numLinks / nLinksToPlot)
        images = []
        for index, links in enumerate(self.linkHistory):

            isNthLink = (index % N == 0)

            if isNthLink:
                for link in links:
                    link.plot(ax)

                # Used to return the plot as an image rray
                # draw the canvas, cache the renderer
                fig.canvas.draw()
                image = np.frombuffer(fig.canvas.tostring_rgb(), dtype='uint8')
                image = image.reshape(fig.canvas.get_width_height()[::-1] +
                                      (3,))
                images.append(image)

        # plotting the end effector
        for endEffectorLocation in self.stateHistory:
            endEffX = endEffectorLocation[0]
            endEffY = endEffectorLocation[1]
            ax.plot(endEffX, endEffY, color='blue', marker='o',
                    linestyle='none', linewidth=4, markersize=9)

        return images

    ##
    # @brief      Function for ManipulatorRobot instance to "run" itself
    #
    #             Runs the planning algorithm, reports if it found a path, and
    #             plots the solution
    #
    # @param      planner    The planner object containing the motion planning
    #                        algorithm to be used on the robot
    # @param      plotTitle  The plot title string
    #
    # @return     runs robot, then plots results of running the robot
    #
    def runAndPlot(self, planner, plotTitle):

        plotConfigData = {'plotTitle': plotTitle + 'wavefrontPlanner',
                          'xlabel': 'theta - link 1 [rad]',
                          'ylabel': 'theta - link 2 [rad]',
                          'plotObstacles': False,
                          'plotGrid': False}
        foundPath = planner.findPathToGoal(startState=self.startCState,
                                           goalState=self.goalCState,
                                           plannerConfigData=None,
                                           plotConfigData=plotConfigData)

        if foundPath:
            print('Reached goal at:', self.stateHistory[-1])
            print('Path length: ', self.distTraveled)
        else:
            print('No valid path found with current planner:',
                  planner.plannerType)

        plotConfigData = {'plotTitle': plotTitle + 'workspace',
                          'xlabel': 'x',
                          'ylabel': 'y'}
        self.workspace.plot(robot=self,
                            startState=self.startState,
                            goalState=self.goalState,
                            plotConfigData=plotConfigData)

        plotConfigData = {'plotTitle': plotTitle + 'cSpace',
                          'xlabel': 'theta - link 1 [rad]',
                          'ylabel': 'theta - link 2 [rad]',
                          'plotObstacles': True,
                          'plotGrid': False}
        self.cSpace.plot(robot=self,
                         startState=self.startCState,
                         goalState=self.goalCState,
                         plotConfigData=plotConfigData)


##
# @brief      Implements the generic builder class for the ManipulatorRobot
#
class ManipulatorRobotBuilder(Builder):

    ##
    # @brief need to call the super class constructor to gain its properties
    #
    def __init__(self):
        Builder.__init__(self)

    ##
    # @brief      Implements the smart constructor for ManipulatorRobot
    #
    #             Only reads the config data once, otherwise just returns the
    #             built object
    #
    # @param      robotType        The ManipulatorRobot type string
    # @param      configFileName   The configuration file name
    # @param      workspace        The Workspace object the ManipulatorRobot
    #                              operates in
    # @param      shouldSavePlots  Boolean controlling whether or not the plt
    #                              objs can be saved to the baseSaveName dir
    # @param      baseSaveFName    The base directory file name for output plot
    #
    # @return     instance of an initialized ManipulatorRobot object
    #
    def __call__(self, robotType, configFileName, workspace, shouldSavePlots,
                 baseSaveFName):

        configNameHasChanged = (self._configName != configFileName)
        noInstanceLoadedYet = (self._instance is None)

        if noInstanceLoadedYet or configNameHasChanged:

            configData = self.loadConfigData(configFileName)
            self._instance = ManipulatorRobot(robotType=robotType,
                                              configData=configData,
                                              workspace=workspace,
                                              shouldSavePlots=shouldSavePlots,
                                              baseSaveFName=baseSaveFName)

        return self._instance
