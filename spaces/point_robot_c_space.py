# 3rd-party packages
import numpy as np
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
    # @param      robot                        The PointRobot instance
    # @param      workspace                    The CSpace object the PointRobot
    #                                          operates in
    # @param      linearDiscretizationDensity  linear discretization density of
    #                                          CSpace
    # @param      shouldSavePlots              Boolean controlling whether or
    #                                          not the plt objs can be saved to
    #                                          the baseSaveName dir
    # @param      baseSaveFName                The base directory file name for
    #                                          output plot
    #
    # @return     initialized PointRobotCSpace object
    #
    def __init__(self, robot, workspace, linearDiscretizationDensity,
                 shouldSavePlots, baseSaveFName):

        # get inherited properties from superclass implementation
        CSpace_2D.__init__(self, robot, workspace, linearDiscretizationDensity,
                           shouldSavePlots, baseSaveFName)

        # need a bounding box for the discretization of CSpace
        (self.minGridX,
         self.maxGridX,
         self.minGridY,
         self.maxGridY) = self.determineSpatialBounds(robot.startState,
                                                      robot.goalState,
                                                      workspace)

        # need to discretize the cspace for the brushfire algorithm
        (self.polygonGridCells,
         self.numericGridCells,
         self.distanceCells,
         self.gridSize) = self.discretizeCSpace(N=linearDiscretizationDensity)

        # compute the distance to an obstacle from any cell using the brushfire
        # algorithm
        (self.distanceCells,
         self.maxManhattanDist) = \
            self.brushFireDistanceComputation(self.distanceCells,
                                              self.polygonGridCells)

        print('Built CSpace with grid cells of size ', self.gridSize)

    ##
    # @brief      Determines an enlarged spatial bounding box for CSpace
    #             enclosing, with margin, the robot and all of its obstacles
    #
    # @param      robotStartState  The robot's start state in cspace
    # @param      robotGoalState   The robot start state
    # @param      workspace        The robot's goal state in cspace
    #
    # @return     (minGridX, maxGridX, minGridY, maxGridY)
    #
    def determineSpatialBounds(self, robotStartState, robotGoalState,
                               workspace):

        # once again, fuck python
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
    # @brief      Implements the brushfire algorithm
    #
    # @param      distCells         The distance cells
    # @param      polygonGridCells  The polygon grid cells
    #
    # @return     (each cell in distCells is labeled with its mimum manhattan
    #              distance from an obstacle,
    #              maximum distance from an obstacle)
    #
    def brushFireDistanceComputation(self, distCells,
                                     polygonGridCells):

        N = self.linearDiscretizationDensity

        # start by computing the set of obstacle cells and their neighbors
        (neighborsOfCellsWithObstacles,
         distCells,
         _) = self.labelObstacleCells(polygonGridCells, distCells, N)

        neighbors = copy.deepcopy(neighborsOfCellsWithObstacles)

        while neighbors:

            neighbor = neighbors.popleft()

            # update distance for all current neighbors
            neighborRow = neighbor[0]
            neighborCol = neighbor[1]
            prevDist = neighbor[2]
            currDist = prevDist + 1
            distCells[neighborRow, neighborCol] = currDist

            possibleNewNeighbors = self.calcNeighbors(neighborRow,
                                                      neighborCol,
                                                      N,
                                                      dist=currDist)

            neighbors = self.updateNeighborCells(distCells,
                                                 possibleNewNeighbors,
                                                 neighbors)

        return (distCells, currDist)

    ##
    # @brief      updates newNeighbors with a list of unvisited neighbors with
    #             the list of given possible new neighbors
    #
    # @param      distanceCells         The distanceCells to update the
    #                                   distance measure in
    # @param      possibleNewNeighbors  The possible new neighbors index list
    # @param      newNeighbors          The neighboring cell index list to add
    #                                   valid neighbors to
    #
    # @return     newNeighbors has unvisited neighbor cells appended
    #
    def updateNeighborCells(self, distanceCells,
                            possibleNewNeighbors, newNeighbors):

        for possibleNewNeighbor in possibleNewNeighbors:

            neighborRow = possibleNewNeighbor[0]
            neighborCol = possibleNewNeighbor[1]
            haveNotVisted = (distanceCells[neighborRow, neighborCol] == 0)

            # the third element of the array is distance, so only compare
            # indices
            isUnique = True
            for newNeighbor in newNeighbors:
                if possibleNewNeighbor[0:2] == newNeighbor[0:2]:
                    isUnique = False

            if haveNotVisted and isUnique:
                newNeighbors.append(copy.deepcopy(possibleNewNeighbor))

        return newNeighbors

    ##
    # @brief      Plots each of the polygon object grid cells onto ax
    #
    # @warning overrides method in superclass
    #
    # @param      ax    matplotlib Axes to plot the grid cells on
    # @param      grid  a list of shapely.Polygon objects representing each
    #                   gridcell
    #
    def plotGrid(self, ax, fig, grid):

        newDist = np.flipud(self.distanceCells)
        x, y = np.fliplr(self.numericGridCells)

        c = ax.pcolor(x, y, newDist,
                      edgecolors='k', linewidths=0,
                      cmap='hot', alpha=0.6)
        cbar = fig.colorbar(c, ax=ax)
        cbar.ax.set_ylabel('manhattan distance from obstacle')


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
    # @param      shouldSavePlots  Boolean controlling whether or not the plt
    #                              objs can be saved to the baseSaveName dir
    # @param      baseSaveFName    The base directory file name for output plot
    # @param      workspace  The CSpace object the PointRobot operates in
    #
    # @return     instance of an initialized PointRobotCSpace object
    #
    def __call__(self, robot, N, shouldSavePlots, baseSaveFName):

        robotHasChanged = (self.robot != robot)
        workspaceHasChanged = (self.workspace != robot.workspace)
        noInstanceLoadedYet = (self._instance is None)

        if noInstanceLoadedYet or robotHasChanged or workspaceHasChanged:

            self._instance = \
                PointRobotCSpace(robot=robot,
                                 workspace=robot.workspace,
                                 linearDiscretizationDensity=N,
                                 shouldSavePlots=shouldSavePlots,
                                 baseSaveFName=baseSaveFName)

        return self._instance
