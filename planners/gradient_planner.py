# 3rd-party packages
import numpy as np
from shapely.geometry import Point
import copy
import matplotlib.pyplot as plt
from scipy.interpolate import griddata

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

        # define the coordinates using CSpace discretization
        self.xCoords, self.yCoords = self.cSpace.numericCoordArrays

        # determines how distance to obstacles should be incorporated:
        #
        # 'perObstacle'
        # calculate minimum distance to each obstacle
        #
        # 'minObstacleDist'
        # use the brushfire algorithm to compute minimum distance to ANY
        # obstacle from each state
        self.distanceMeasurement = configData['distanceMeasurement']

        # list of critical potential values for each obstacle
        self.qStars = configData['qStars']

        # threshold distance from goal for attractive potential
        self.dStarGoal = configData['dStarGoal']

        # goal region
        self.goalEpsilon = configData['goalEpsilon']

        # attractive potential gain
        self.attPotGain = configData['attractivePotentialGain']

        # repulsive potential gain
        self.repPotGain = configData['repulsivePotentialGain']

        # determine step size of the gradient descent algorithm
        self.stepSize = configData['stepSize']

    ##
    # @brief      Computes a viable path in robot's cSpace to the goalState
    #             from the robot's startState
    #
    # @param      plotTitle  The plot title string
    #
    # @return     a viable set of cSpace states from startState to goalState
    #
    def findPathToGoal(self, plotTitle):

        U, points = self.calcPotentialField()

        print('Starting gradient descent planner using distance measure: ',
              self.distanceMeasurement)

        foundPath = self.gradientDescent(U, points)
        self.plotPotentialField(U=U, plotTitle=plotTitle)

        return foundPath

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
        distToGoal = robot.distToTarget(state, robot.goalState.flatten())

        # unroll for speed
        gain = self.attPotGain
        dStarGoal = self.dStarGoal

        if distToGoal <= dStarGoal:

            U_att = 0.5 * gain * distToGoal ** 2

        else:

            U_att = dStarGoal * gain * distToGoal - \
                0.5 * gain * dStarGoal ** 2

        return U_att

    ##
    # @brief      defines the repulsive potential field value at a given
    #             configuration state
    #
    # @param      state                The configuration state state
    # @param      distanceMeasurement  determines how distance to obstacles
    #                                  should be incorporated:
    #                                  - perObstacle  calculate minimum
    #                                    distance to each obstacle
    #                                  - minObstacleDist   use the brushfire
    #                                    algorithm to compute minimum distance
    #                                    to ANY obstacle from each state
    #
    # @return     the repulsive potential field value at state
    #
    def repulsivePotential(self, state, distanceMeasurement='perObstacle'):

        gain = self.repPotGain

        if distanceMeasurement == 'perObstacle':

            obstacles = self.cSpace.polygonObstacles

            # repulsive potential is the sum of repulsive potential
            # contributions from each obstacle
            U_rep = 0

            for (obstacle, qStar) in zip(obstacles, self.qStars):

                distToObst = abs(obstacle.exterior.distance(Point(state)))

                # return nan, as we collided with the obstacle
                if distToObst == 0:
                    distToObst = np.nan

                if distToObst <= qStar:
                    U_rep += 0.5 * gain * (1 / distToObst - 1 / qStar) ** 2
                else:
                    U_rep += 0

        elif distanceMeasurement == 'minObstacleDist':

            (row, col) = self.cSpace.getGridCoordsFromState(state)

            minDistToObst = self.cSpace.distanceCells[row, col]
            qStar = self.qStars[0]

            # return nan, as we collided with the obstacle
            if minDistToObst < 1:
                return np.nan

            if minDistToObst <= qStar:
                U_rep = 0.5 * gain * (1 / qStar - 1 / minDistToObst) ** 2
            else:
                U_rep = 0

        else:

            raise ValueError(distanceMeasurement)

        return U_rep

    ##
    # @brief      Defines the total potential field value at a given state
    #
    # @param      state  The state
    #
    # @return     the potential field value at state
    #
    def potential(self, state):

        return (self.attractivePotential(state) +
                self.repulsivePotential(state, self.distanceMeasurement))

    ##
    # @brief      Calculates the potential field over CSpace
    #
    # @return     The potential field 2D array
    #
    def calcPotentialField(self):

        xCoords = self.xCoords
        yCoords = self.yCoords

        shape = xCoords.shape
        nCoordsX = shape[0]
        shape = yCoords.shape
        nCoordsY = shape[0]
        U = np.zeros((nCoordsY, nCoordsX))
        points = []

        for i_x, x in enumerate(xCoords):
            for i_y, y in enumerate(yCoords):

                state = copy.deepcopy(np.array([x, y]))
                potential = self.potential(state)
                U[i_y, i_x] = potential

                points.append((x, y))

        return U, points

    ##
    # @brief      Determines if robot is at goal.
    #
    # @return     True if at goal, False otherwise.
    #
    def isAtGoal(self):

        closeToGoal = self.isCloseTo(self.goalState, self.goalEpsilon)

        return closeToGoal

    ##
    # @brief      Determines if the robot is close to a given target location
    #
    # @param      targetLocation  The target location
    # @param      epsilon         The location matchin epsilon radius
    #
    # @return     True if the robot is close to target, False otherwise.
    #
    def isCloseTo(self, targetLocation, epsilon):

        distToLocation = self.robot.distToTarget(self.robot.currentState,
                                                 targetLocation)

        return (distToLocation <= epsilon)

    ##
    # @brief      runs the gradient descent algorithm to get a path in cspace
    #             from the robot's start to goal states
    #
    # @param      U       the potential field over the discretized grid
    # @param      points  a list of x-y coordinate tuples for each elem of U
    #
    # @return     the viable sequence of states to goal
    #
    def gradientDescent(self, U, points):

        # define gradient magnitude tolerance for being at a local minima
        minimaTol = 1e-4

        # define maximum iterations to try
        nIter = 500

        robot = self.robot

        states = []
        state = copy.deepcopy(self.robot.startState)
        robot.updateRobotState(copy.deepcopy(state))
        states.append(copy.deepcopy(state))

        # calculate the gradient on the CSpace grid fist
        dy, dx = np.gradient(U, self.cSpace.yGridSize, self.cSpace.xGridSize)

        # Making your own gradient interpolater is SUPER fun :)))
        # have to flatten in column order (order='C' for row order lol wtf)
        dx_values = dx.flatten(order='F')
        dy_values = dy.flatten(order='F')

        iterCount = 0
        updateRate = 20
        while not self.isAtGoal():

            interp_gradient = np.zeros((2, 1))
            currX, currY = state

            # at each new query point, interpolate the gradient smoothly
            interp_dx = griddata(points, dx_values, (currX, currY),
                                 method='cubic')
            interp_dy = griddata(points, dy_values, (currX, currY),
                                 method='cubic')
            interp_gradient[0] = self.stepSize * interp_dx
            interp_gradient[1] = self.stepSize * interp_dy

            # give user status of solution
            shouldPrint = (iterCount % updateRate == 0)
            if shouldPrint:
                print('state: ', state, 'dx: ', interp_dx, 'dy:', interp_dy)

            iterCount += 1

            state -= interp_gradient
            robot.updateRobotState(copy.deepcopy(state))
            states.append(copy.deepcopy(state))

            # if the descent reached nans, it went out of bounds of the
            # original bounded cspace region -> planner settings failed
            #
            # if derivatives are super small, it failed
            hitObstacle = any([np.isnan(stateCoord[0])
                               for stateCoord in state])
            atLocalMinima = ((np.linalg.norm(interp_gradient) < minimaTol) and
                             not self.isAtGoal())
            outOfIterations = iterCount >= nIter

            if hitObstacle or atLocalMinima or outOfIterations:
                return False

        return True

    #
    # @brief      plots the potential field overlaid on the cspace plot
    #
    # @param      U          the potential field 2d array
    # @param      plotTitle  The plot title string
    #
    def plotPotentialField(self, U, plotTitle):

        potField = U

        xGrid, yGrid = self.cSpace.numericGridCells

        x = xGrid
        y = yGrid
        dy, dx = np.gradient(potField, self.cSpace.yGridSize,
                             self.cSpace.xGridSize)

        N = self.cSpace.N

        fig = plt.figure()
        ax = fig.add_subplot(111)

        cs = ax.contourf(x, y, potField, N, alpha=0.7)
        plt.quiver(x, y, dx, dy, units='width')
        cbar = fig.colorbar(cs, ax=ax, orientation="vertical")
        cbar.ax.set_ylabel('potential function value')

        plotConfigData = {'plotTitle': plotTitle + 'gradientPlanner',
                          'xlabel': 'x',
                          'ylabel': 'y',
                          'plotGrid': False}
        self.cSpace.plot(robot=self.robot,
                         startState=self.robot.startState,
                         goalState=self.robot.goalState,
                         plotConfigData=plotConfigData,
                         ax=ax, fig=fig)


##
# @brief      Implements the generic builder class for the GradientPlanner
#
class GradientPlannerBuilder(Builder):

    ##
    # @brief      need to call the super class constructor to gain its
    #             properties
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
