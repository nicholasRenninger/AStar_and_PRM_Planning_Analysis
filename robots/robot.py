# 3rd-party packages
import numpy as np


##
# @brief      This (largely abstract) class describes a generic robot
#
class Robot:

    ##
    # @brief      Robot class constructor
    #
    # @param      robotType        The robot type string
    # @param      configData       Configuration dictionary for the robot
    # @param      workspace        The Workspace object the robot operates in
    # @param      shouldSavePlots  Boolean controlling whether or not the plt
    #                              objs can be saved to the baseSaveName dir
    # @param      baseSaveFName    The base directory file name for output plot
    # @param      self  The Robot object object
    #
    # @return     initialized Robot object
    #
    def __init__(self, robotType, configData, workspace,
                 shouldSavePlots, baseSaveFName):

        self.workspace = workspace
        self.cSpace = None
        self.robotType = robotType

        # state refers to the robot's actual state vector
        self.startState = None
        self.goalState = None
        self.distTraveled = None
        self.stateHistory = None
        self.currentState = None
        self.initializeRobotState(configData)

    ##
    # @brief      returns the start state and goal location lists for the robot
    #
    # @param      configData  configuration data dictionary for the robot
    #
    # @return     The start and goal states lists from configData
    #
    def initializeRobotState(self, configData):

        startStateList = configData['startState']
        goalStateList = configData['goalState']

        numStates = len(startStateList)

        self.startState = np.array(startStateList,
                                   dtype='float64').reshape((numStates, 1))
        self.goalState = np.array(goalStateList,
                                  dtype='float64').reshape((numStates, 1))

        self.distTraveled = 0
        self.stateHistory = []
        self.currentState = None

    ##
    # @brief      abstract method to set the robot shape from the configData
    #
    # @param      configData  configuration data dictionary for the robot
    #
    # @return     robot's shape data in the concrete class will be set
    #
    def setRobotShape(self, configData):

        raise NotImplementedError

    ##
    # @brief      abstract method to check if a state in Robot's config space
    #             collides with a workspace obstacle
    #
    # Needs to be implemented for each specific type of robot / workspace combo
    #
    # @param      state  The robot configuration space state to check
    #
    # @return     Bool indicating whether the state is in collision
    #
    def checkCollision(self, state):

        raise NotImplementedError

    ##
    # @brief      computes the distance to the goal state
    #
    # @param      self  The Robot object
    #
    # @return     the distance to the goal state of the robot
    #
    def distToGoal(self):

        dist = self.distToTarget(self.goalState)

        return dist

    ##
    # @brief      computes the distance to the target
    #
    # @param      target  The target coordinates
    # @param      self  The Robot object
    #
    # @return     the distance to the target object
    #
    def distToTarget(self, currentState, target):

        distVec = target - currentState
        distToLocation = np.linalg.norm(distVec)

        return distToLocation

    ##
    # @brief      Virtual member function to plot the robot in the workspace
    #
    # @param      ax    the matplotlib.axes object to plot the robot's body in
    #
    # @return     plots the robot's body on ax
    #
    def plotInWorkspace(self, ax):

        raise NotImplementedError

    ##
    # @brief      virtual member function for a robot instance to "run" itself
    #
    # @param      planner          The Planners object containing the motion
    #                              planning algorithm to be used on the robot
    # @param      shouldSavePlots  Boolean to turn on and off plot file writes
    # @param      baseSaveFName    The base save file name for plots
    #
    # @return     runs robot, then plots results of running the robot
    #
    def runAndPlot(self, planner, shouldSavePlots, baseSaveFName):

        raise NotImplementedError

    ##
    # @brief      Calculates a unit vector heading from currentPosition to
    #             targetObjectPosition
    #
    # @param      currentPosition       The current position
    # @param      targetObjectPosition  The target object position
    #
    # @return     The heading unit vector between currentPosition and
    #             targetObjectPosition
    #
    def computeHeading(self, currentPosition, targetObjectPosition):

        headingVecUnNorm = targetObjectPosition - currentPosition

        # need to normalize the heading to get a unit vector heading
        headingVec = headingVecUnNorm / np.sqrt(np.sum(headingVecUnNorm ** 2))

        return headingVec
