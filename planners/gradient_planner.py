# 3rd-party packages
import numpy as np

# local packages
from planners.planner import Planner
from factory.builder import Builder


##
# @brief      This class describes a Planner subclass used to find a path
#             in two-dimensional configuration space using gradient descent
#             algorithms
#
class GradientPlanner(Planner):

    ##
    # @brief      GradientPlanner class constructor
    #
    # @param      plannerType      The planner type string
    # @param      cSpace           The configuration space of the robot
    # @param      workspace        The workspace object the robot operates in
    # @param      robot            The Planner type string
    # @param      configData       Configuration dictionary for the planner
    # @param      shouldSavePlots  Boolean controlling whether or not the plt
    #                              objs can be saved to the baseSaveName dir
    # @param      baseSaveFName    The base directory file name for output plot
    #
    # @return     initialized Planner object
    #
    def __init__(self, plannerType, cSpace, workspace, robot, configData,
                 shouldSavePlots, baseSaveFName):

        # calling the superclass contructor to inherit its properties
        Planner.__init__(self,
                         plannerType=plannerType,
                         cSpace=cSpace,
                         workspace=workspace,
                         robot=robot,
                         configData=configData,
                         shouldSavePlots=shouldSavePlots,
                         baseSaveFName=baseSaveFName)

        # list of critical potential values for each obstacle
        self.qStars = configData['qStars']

        # threshold distance from goal for attractive potential
        self.dStarGoal = configData['dStarGoal']

        # goal region
        self.goalEpsilon = configData['goalEpsilon']

        # attractive potential scaling factor
        self.attPotScaleFactor = configData['attractivePotentialScalingFactor']

    ##
    # @brief      Computes a viable path in robot's cSpace to the goalState
    #             from the robot's startState
    #
    # @return     a viable set of cSpace states from startState to goalState
    #
    def findPathToGoal(self):

        return self.gradientDescent()

    ##
    # @brief      Gets the distance to the closest obstacle from a given state
    #
    # @param      state  The robot configuration state
    #
    # @return     the manhattan distance to the closest obstacle
    #
    def getClosestObstacleDist(self, state):

        row, col = self.cSpace.getGridCoordsFromState(state)
        distance = self.cSpace.distanceCells[row, col]

        return distance

    ##
    # @brief      defines the attractive potential field value at a given
    #             configuration state
    #
    # @param      state  The configuration state of the robot
    #
    # @return     the attractive potential field value at state
    #
    def attractivePotential(self, state):

        robot = self.robot
        distToGoal = robot.distToGoal()

        # unroll for speed
        attPotScaleFactor = self.attPotScaleFactor
        dStarGoal = self.dStarGoal

        if distToGoal <= dStarGoal:

            U_att = 0.5 * attPotScaleFactor * distToGoal ** 2

        else:

            U_att = dStarGoal * attPotScaleFactor * distToGoal - \
                0.5 * attPotScaleFactor * dStarGoal ** 2

        return U_att

    ##
    # @brief      defines the repulsive potential field value at a given
    #             configuration state
    #
    # @param      state  The configuration state state
    #
    # @return     the repulsive potential field value at state
    #
    def repulsivePotential(self, state):

        obstacles = self.robot.cSpace.obstacles

        for obstacle in obstacles:
            pass

    ##
    # @brief      Defines the total potential field value at a given state
    #
    # @param      state  The state
    #
    # @return     the potential field value at state
    #
    def potential(self, state):

        return self.attractivePotential(state) + self.repulsivePotential(state)

    ##
    # @brief      runs the gradient descent algorithm to get a path in cspace
    #             from the robot's start to goal states
    #
    def gradientDescent(self):

        pass

    #
    # @brief      plots the potential field overlaid on the cspace
    #
    def plotPotentialField(self, plotTitle):

        ax = self.cSpace.plot(robot=self.robot,
                              startState=self.startState,
                              goalState=self.goalState,
                              plotTitle=plotTitle + 'potentialField')


##
# @brief      Implements the generic builder class for the GradientPlanner
#
class GradientPlannerBuilder(Builder):

    ##
    # @brief need to call the super class constructor to gain its properties
    #
    def __init__(self):
        Builder.__init__(self)
        self.robot = None
        self.workspace = None

    ##
    # @brief      Implements the smart constructor for GradientPlanner
    #
    #             Only reads the config data once, otherwise just returns the
    #             built object
    #
    # @param      plannerType      The planner type string
    # @param      cSpace           The configuration space of the robot
    # @param      workspace        The workspace object the robot operates in
    # @param      robot            The Planner type string
    # @param      configFileName   The configuration file name
    # @param      shouldSavePlots  Boolean controlling whether or not the plt
    #                              objs can be saved to the baseSaveName dir
    # @param      baseSaveFName    The base directory file name for output plot
    #
    # @return     instance of an initialized GradientPlanner object
    #
    def __call__(self, plannerType, cSpace, workspace, robot, configFileName,
                 shouldSavePlots, baseSaveFName):

        robotHasChanged = (self.robot != robot)
        workspaceHasChanged = (self.workspace != workspace)
        configNameHasChanged = (self._configName != configFileName)
        noInstanceLoadedYet = (self._instance is None)

        if noInstanceLoadedYet or robotHasChanged or workspaceHasChanged or \
           configNameHasChanged:

            configData = self.loadConfigData(configFileName)
            self._instance = GradientPlanner(plannerType=plannerType,
                                             cSpace=cSpace,
                                             workspace=workspace,
                                             robot=robot,
                                             configData=configData,
                                             shouldSavePlots=shouldSavePlots,
                                             baseSaveFName=baseSaveFName)

        return self._instance
