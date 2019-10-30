# 3rd-party packages
import copy

# local packages
from factory.builder import Builder
from spaces.cSpace_2d import CSpace_2D


##
# @brief      Concrete implementation of the 2D cspace for a point robot
#
class PointRobotCSpace(CSpace_2D):

    ##
    # @brief      PointRobotCSpace class constructor
    #
    # @param      robot            The PointRobot instance
    # @param      workspace        The CSpace object the PointRobot operates in
    # @param      N                linear discretization density of CSpace
    # @param      makeSquare       determines whether the CSpace will be made
    #                              square when enlarged for planning. for the
    #                              gradient descent planner, you must set this
    #                              to be True
    # @param      shouldSavePlots  Boolean controlling whether or not the plt
    #                              objs can be saved to the baseSaveName dir
    # @param      baseSaveFName    The base directory file name for output plot
    #
    # @return     initialized PointRobotCSpace object
    #
    def __init__(self, robot, workspace, N, makeSquare,
                 shouldSavePlots, baseSaveFName):

        # get inherited properties from superclass implementation
        CSpace_2D.__init__(self, robot, workspace, N,
                           shouldSavePlots, baseSaveFName)

        # as the robot is a point robot, its obstacles are just the workspace
        # obstacles
        self.obstacles = workspace.obstacles
        self.polygonObstacles = workspace.polygonObstacles

        # need a bounding box for the discretization of CSpace
        (self.minGridX,
         self.maxGridX,
         self.minGridY,
         self.maxGridY) = self.determineSpatialBounds(robot.startState,
                                                      robot.goalState,
                                                      workspace, makeSquare)

        # need to discretize the cspace for the brushfire algorithm
        (self.polygonGridCells,
         self.numericGridCells,
         self.numericCoordArrays,
         initDistanceCells,
         (self.xGridSize,
          self.yGridSize)) = self.discretizeCSpace(N)

        # save the initial distance cells to be used by the wavefront planner
        self.initDistanceCells = initDistanceCells

        # compute the distance to an obstacle from any cell using the brushfire
        # algorithm
        (self.distanceCells,
         self.maxManhattanDist) = \
            self.brushFireDistanceComputation(copy.deepcopy(initDistanceCells),
                                              self.polygonGridCells)

        print('Built CSpace with grid cells of size :',
              self.xGridSize, ' x ', self.yGridSize)

    ##
    # @brief      Determines an enlarged spatial bounding box for CSpace
    #             enclosing, with margin, the robot and all of its obstacles
    #
    # @param      robotStartState  The robot's start state in cspace
    # @param      robotGoalState   The robot start state
    # @param      workspace        The robot's goal state in cspace
    # @param      makeSquare       determines whether the CSpace will be made
    #                              square when enlarged for planning. for the
    #                              gradient descent planner, you must set this
    #                              to be True
    #
    # @return     (minGridX, maxGridX, minGridY, maxGridY)
    #
    def determineSpatialBounds(self, robotStartState, robotGoalState,
                               workspace, makeSquare):

        # once again, (0_0) python
        robStartX = copy.deepcopy(robotStartState[0])
        robStartY = copy.deepcopy(robotStartState[1])
        robGoalX = copy.deepcopy(robotGoalState[0])
        robGoalY = copy.deepcopy(robotGoalState[1])

        # cspace bounding box should include the robot's possible positions as
        # well as the obstacle positions maxima
        minGridX = min(workspace.minXObst, robStartX, robGoalX)
        maxGridX = max(workspace.maxXObst, robStartX, robGoalX)
        minGridY = min(workspace.minYObst, robStartY, robGoalY)
        maxGridY = max(workspace.maxYObst, robStartY, robGoalY)

        # now extra space needs to be added around the bounding box to allow
        # the robot to move freely in space. Have to check signs here to ensure
        # that the space is actually expanded lolol
        maxGridCoord = max(minGridX, maxGridX,
                           minGridY, maxGridY)
        scale = 0.2

        minGridX = self.enlargeBBCoord(gridCoord=minGridX,
                                       maxGridCoord=maxGridCoord,
                                       scale=scale,
                                       coordType='min')
        maxGridX = self.enlargeBBCoord(gridCoord=maxGridX,
                                       maxGridCoord=maxGridCoord,
                                       scale=scale,
                                       coordType='max')
        minGridY = self.enlargeBBCoord(gridCoord=minGridY,
                                       maxGridCoord=maxGridCoord,
                                       scale=scale,
                                       coordType='min')
        maxGridY = self.enlargeBBCoord(gridCoord=maxGridY,
                                       maxGridCoord=maxGridCoord,
                                       scale=scale,
                                       coordType='max')

        # only renormalize the grid bounding box to be square if desired
        if makeSquare:

            length = abs(maxGridX - minGridX)
            height = abs(maxGridY - minGridY)

            if length > height:
                nonSquareness = (length - height) / 2
                maxGridY += nonSquareness
                minGridY -= nonSquareness

            elif length < height:
                nonSquareness = (height - length) / 2
                maxGridX += nonSquareness
                minGridX -= nonSquareness

        return (float(minGridX), float(maxGridX),
                float(minGridY), float(maxGridY))

    ##
    # @brief      Appropriately scales a grid coordinate depending on its sign
    #             and whether it's a maximum grid coordinate
    #
    # @param      gridCoord     The grid coordinate to scale
    # @param      maxGridCoord  The maximum grid coordinate of all cardinal
    #                           axes, used to expand an extrema grid coordinate
    #                           which is 0
    # @param      scale         The scaling factor for each coordinate
    # @param      coordType     The coordinate type string
    #
    # @return     the scaled gridCoord
    #
    def enlargeBBCoord(self, gridCoord, maxGridCoord, scale, coordType):

        if coordType == 'min':

            if gridCoord != 0:
                gridCoord -= abs(gridCoord) * scale
            else:
                # in case the min grid coord is zero, expand it using the
                # largest other coordinate on the bounding box
                gridCoord -= abs(maxGridCoord) * scale

        elif coordType == 'max':

            if gridCoord != 0:
                gridCoord += abs(gridCoord) * scale
            else:
                # in case the min grid coord is zero, expand it using the
                # largest other coordinate on the bounding box
                gridCoord += abs(maxGridCoord) * scale

        else:
            raise ValueError(coordType)

        return gridCoord


##
# @brief      Implements the generic builder class for the PointRobotCSpace
#
class PointRobotCSpaceBuilder(Builder):

    # need to call the super class constructor to gain its properties
    #
    def __init__(self):
        Builder.__init__(self)
        self.robot = None
        self.workspace = None

    ##
    # @brief      Implements the smart constructor for PointRobotCSpace
    #
    #             Only reads the config data once, otherwise just returns the
    #             built object
    #
    # @param      robot            The PointRobotCSpace type string
    # @param      N                linear discretization density of CSpace
    # @param      makeSquare       determines whether the CSpace will be made
    #                              square when enlarged for planning. for the
    #                              gradient descent planner, you must set this
    #                              to be True
    # @param      shouldSavePlots  Boolean controlling whether or not the plt
    #                              objs can be saved to the baseSaveName dir
    # @param      baseSaveFName    The base directory file name for output plot
    # @param      workspace  The CSpace object the PointRobot operates in
    #
    # @return     instance of an initialized PointRobotCSpace object
    #
    def __call__(self, robot, N, makeSquare, shouldSavePlots, baseSaveFName):

        robotHasChanged = (self.robot != robot)
        workspaceHasChanged = (self.workspace != robot.workspace)
        noInstanceLoadedYet = (self._instance is None)

        if noInstanceLoadedYet or robotHasChanged or workspaceHasChanged:

            self._instance = \
                PointRobotCSpace(robot=robot,
                                 workspace=robot.workspace,
                                 N=N,
                                 makeSquare=makeSquare,
                                 shouldSavePlots=shouldSavePlots,
                                 baseSaveFName=baseSaveFName)

        return self._instance
