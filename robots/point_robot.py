# 3rd-party packages
import copy
from shapely.geometry import Point

# local packages
from robots.robot import Robot
from factory.builder import Builder
from spaces.factory import activeSpaces


##
# @brief      This class describes a point Robot subclass that has no physical
#             dimension in the workspace
#
class PointRobot(Robot):

    ##
    # @brief      PointRobot class constructor
    #
    # @param      robotType        The PointRobot type string
    # @param      configData       Configuration dictionary for the
    #                              PointRobot
    # @param      workspace        The Workspace object the PointRobot
    #                              operates in
    # @param      shouldSavePlots  Boolean controlling whether or not the plt
    #                              objs can be saved to the baseSaveName dir
    # @param      baseSaveFName    The base directory file name for output plot
    #
    # @return     initialized PointRobot object
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

        linearDiscretizationDensity = configData['linearDiscretizationDensity']

        # have the factory make the PointRobot's C-space
        self.cSpace = activeSpaces.get(robotSpaceType='POINTROBOTCSPACE',
                                       robot=self,
                                       N=linearDiscretizationDensity,
                                       shouldSavePlots=shouldSavePlots,
                                       baseSaveFName=baseSaveFName)

    ##
    # @brief      Returns whether the given Shapely object collides with any
    #             workspace obstacles
    #
    # @param      shapelyObject  The shapely object to check for collision
    #
    # @return     True if the hapely object collides with any obstacle, False
    #             otherwise
    #
    def checkCollision(self, shapelyObject):

        obstacles = self.workspace.polygonObstacles
        for obstacle in obstacles:
            if obstacle.intersects(shapelyObject):
                return True

        return False

    ##
    # @brief      Returns whether the CSpace state collides with any workspace
    #             obstacles
    #
    # @param      state  The configuration space state as a numpy list
    #
    # @return     True if the CSpace state collides with any obstacle, False
    #             otherwise
    #
    def checkCollisionWithState(self, state):

        pointToCheck = Point(state)
        self.checkCollision(pointToCheck)

        return False

    ##
    # @brief      Puts the robot into the newState and updates appropriate data
    #
    # @param      newState  The new state for the PointRobot
    #
    def updateRobotState(self, newState):

        self.currentState = newState
        self.stateHistory.append(copy.deepcopy(newState))
        self.distTraveled += self.distToTarget(newState)

    ##
    # @brief      Plots the robot's body's path in the workspace
    #
    # @note       this function does nothing, as the PointRobot has no body
    #
    # @param      ax    the matplotlib.axes object to plot the PointRobot's
    #                   body's path in its Workspace
    #
    # @return     plots the PointRobot's body's path on ax
    #
    def plotBodyInWorkspace(self, ax):
        pass

    ##
    # @brief      Function for PointRobot instance to "run" itself
    #
    #             For the PointRobot, we are just going to linearly evolve
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
# @brief      Implements the generic builder class for the PointRobot
#
class PointRobotBuilder(Builder):

    ##
    # @brief need to call the super class constructor to gain its properties
    #
    def __init__(self):
        Builder.__init__(self)

    ##
    # @brief      Implements the smart constructor for PointRobot
    #
    #             Only reads the config data once, otherwise just returns the
    #             built object
    #
    # @param      robotType        The PointRobot type string
    # @param      configFileName   The configuration file name
    # @param      workspace        The Workspace object the PointRobot
    #                              operates in
    # @param      shouldSavePlots  Boolean controlling whether or not the plt
    #                              objs can be saved to the baseSaveName dir
    # @param      baseSaveFName    The base directory file name for output plot
    #
    # @return     instance of an initialized PointRobot object
    #
    def __call__(self, robotType, configFileName, workspace, shouldSavePlots,
                 baseSaveFName):

        configNameHasChanged = (self._configName != configFileName)
        noInstanceLoadedYet = (self._instance is None)

        if noInstanceLoadedYet or configNameHasChanged:

            configData = self.loadConfigData(configFileName)
            self._instance = PointRobot(robotType=robotType,
                                        configData=configData,
                                        workspace=workspace,
                                        shouldSavePlots=shouldSavePlots,
                                        baseSaveFName=baseSaveFName)

        return self._instance
