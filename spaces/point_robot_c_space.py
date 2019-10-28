# 3rd-party packages
import matplotlib.pyplot as plt
import numpy as np
from shapely.geometry import Polygon
import copy
from collections import deque as deque
from scipy import spatial
import pyresample
import math

# local packages
from factory.builder import Builder
from spaces.robot_space import RobotSpace


##
# @brief      Concrete implementation of the 2D cspace for a point robot
#
class PointRobotCSpace(RobotSpace):

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
        RobotSpace.__init__(self, shouldSavePlots, baseSaveFName)

        self.workspace = robot.workspace
        self.robot = robot

        # as the robot is a point robot, its obstacles are just
        self.obstacles = robot.workspace.obstacles

        # need a bounding box for the discretization of CSpace
        (self.minGridX,
         self.maxGridX,
         self.minGridY,
         self.maxGridY) = self.determineSpatialBounds(robot.startState,
                                                      robot.goalState,
                                                      workspace)

        # need to discretize the cspace for the brushfire algorithm
        (pCells,
         (nXCells, nYCells),
         distCells,
         self.gridSize) = self.discretizeCSpace(N=linearDiscretizationDensity)

        self.polygonGridCells = pCells
        self.numericGridCells = (nXCells, nYCells)
        self.linearDiscretizationDensity = linearDiscretizationDensity

        # compute the distance to an obstacle from any cell using the brushfire
        # algorithm
        (self.distanceCells,
         self.maxManhattanDist) = self.brushFireDistanceComputation(distCells,
                                                                    pCells)
        print(self.distanceCells[self.getGridCoordsFromState([0, 0])])
        print(self.distanceCells[self.getGridCoordsFromState([3, 0])])
        print(self.distanceCells[self.getGridCoordsFromState([8, 0])])
        print(self.distanceCells[self.getGridCoordsFromState([14, 0])])
        print(self.distanceCells[self.getGridCoordsFromState([0, 4])])
        print(self.distanceCells[self.getGridCoordsFromState([3, 4])])
        print(self.distanceCells[self.getGridCoordsFromState([8, 4])])
        print(self.distanceCells[self.getGridCoordsFromState([14, 4])])
        print(self.distanceCells[self.getGridCoordsFromState([0, 8])])
        print(self.distanceCells[self.getGridCoordsFromState([3, 8])])
        print(self.distanceCells[self.getGridCoordsFromState([8, 8])])
        print(self.distanceCells[self.getGridCoordsFromState([14, 8])])
        print(self.distanceCells[self.getGridCoordsFromState([0, 14])])
        print(self.distanceCells[self.getGridCoordsFromState([3, 14])])
        print(self.distanceCells[self.getGridCoordsFromState([8, 14])])
        print(self.distanceCells[self.getGridCoordsFromState([14, 14])])
        print('Built CSpace with grid cells of size ', self.gridSize)

    ##
    # @brief      Plots all obstacles in the cspace to ax
    #
    # @param      ax    the matplotlib.axes object to plot the obstacles on
    #
    def plotObstacles(self, ax):

        for obstacle in self.obstacles:
            obstX, obstY = zip(*obstacle)
            ax.fill(obstX, obstY,
                    facecolor='black',
                    edgecolor='black',
                    linewidth=1)

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
    # @brief      discretizes CSpace into an NxN grid, based on the bounding
    #             box for CSpace
    #
    # @param      N     number of grid points in each CSpace dimension
    #
    # @return     (a list of polygon objects - one for each grid cell, a numpy
    #             meshgrid representing the grid coordinates, a 2D numpy array
    #             of zeros used  brushfire algorithms, the size of each grid
    #             cell)
    #
    def discretizeCSpace(self, N):

        xCoords = np.linspace(self.minGridX, self.maxGridX, N + 1)
        yCoords = np.linspace(self.minGridY, self.maxGridY, N + 1)

        gridSize = abs(xCoords[1] - xCoords[0])

        xMesh, yMesh = np.meshgrid(yCoords, xCoords)
        emptyDistanceCells = np.zeros((N, N))

        # need to store the grid as an indexed set of cells for brushfire need
        # to use a dictionary, lists of lists are reference traps e.g.
        # FUUUUUUUCCCCCKKKKKKKKK PYYTTTHHHOON
        #
        # Here, we want the i index to
        # refer to the row of the cell, and j to correspond to the column of
        # the cell
        #
        # j = 0, i = 0: bottom left corner of grid
        polygonGridCells = {}
        for i in range(N):
            for j in range(N):

                cell = [(xMesh[j][i], yMesh[j][i]),
                        (xMesh[j][i + 1], yMesh[j][i + 1]),
                        (xMesh[j + 1][i + 1], yMesh[j + 1][i + 1]),
                        (xMesh[j + 1][i], yMesh[j + 1][i])]

                cellPolygon = Polygon(cell)
                polygonGridCells[(j, i)] = copy.deepcopy(cellPolygon)

        return (polygonGridCells, (xMesh, yMesh), emptyDistanceCells, gridSize)

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
    # @brief      labeling all cells intersecting with obstacles as having
    #             distance 1
    #
    # @param      polygonGridCells  The polygon object grid cells
    # @param      distCells         The distance calculation cells
    # @param      N                 linear discretization density of CSpace
    #
    # @return     (a unique list of indices of neighbors of the obstacle cells,
    #              distCells with a 1 at all cells intersecting an obstacle,
    #              the maximum distance explored, currDist)
    #
    def labelObstacleCells(self, polygonGridCells, distCells, N):

        currDist = 1
        neighborsOfCellsWithObstacles = deque()
        obstacleLocations = []
        for i in range(N):
            for j in range(N):

                currCell = polygonGridCells[(i, j)]
                obstacleInCell = self.robot.checkCollision(currCell)

                if obstacleInCell:
                    obstacleLocations.append((i, j))
                    distCells[i, j] = currDist

        # can only do proper neighbor calculation once all obstacles have
        # been added to the distCell grid
        for i, j in obstacleLocations:

            possibleNeighbors = self.calcNeighbors(i, j, N, currDist)

            neighborsOfCellsWithObstacles = \
                self.updateNeighborCells(distCells,
                                         possibleNeighbors,
                                         neighborsOfCellsWithObstacles)

        return (neighborsOfCellsWithObstacles, distCells, currDist)

    ##
    # @brief      Calculates the indices of all possible neighboring cells at
    #             (i, j)
    #
    # @param      i     row index
    # @param      j     column index
    # @param      N     number of rows / cols
    # @param      dist  The distance that the previous neighbor was at
    #
    # @return     a list of all possible valid neighbor coordinates
    #
    def calcNeighbors(self, i, j, N, dist):

        if i == 0 and j == 0:
            # bottom left corner
            neighbors = [(i + 1, j, dist), (i, j + 1, dist)]

        elif i == 0 and j == N - 1:
            # bottom right corner
            neighbors = [(i + 1, j, dist), (i, j - 1, dist)]

        elif i == N - 1 and j == 0:
            # top left corner
            neighbors = [(i - 1, j, dist), (i, j + 1, dist)]

        elif i == N - 1 and j == N - 1:
            # top right corner
            neighbors = [(i - 1, j, dist), (i, j - 1, dist)]

        elif i == 0:
            # bottom edge
            neighbors = [(i, j + 1, dist), (i, j - 1, dist), (i + 1, j, dist)]

        elif i == N - 1:
            # top edge
            neighbors = [(i, j + 1, dist), (i, j - 1, dist), (i - 1, j, dist)]

        elif j == 0:
            # left edge
            neighbors = [(i + 1, j, dist), (i - 1, j, dist), (i, j + 1, dist)]

        elif j == N - 1:
            # top edge
            neighbors = [(i + 1, j, dist), (i - 1, j, dist), (i, j - 1, dist)]

        else:
            # center
            neighbors = [(i + 1, j, dist), (i - 1, j, dist),
                         (i, j + 1, dist), (i, j - 1, dist)]

        return neighbors

    ##
    # @brief      Gets the grid coordinates in the discretized cspace from
    #             state
    #
    # @param      state  The queried state coordinate
    #
    # @return     (row of self.distanceCells corresponding to the state,
    #              column of self.distanceCells corresponding to the state)
    #
    def getGridCoordsFromState(self, state):

        # need to convert numpy array for state into list of coordinates
        # stateCoords = np.concatenate(state, axis=0)
        stateCoords = state
        qX = stateCoords[0]
        qY = stateCoords[1]

        col = math.floor((qX - self.maxGridX) / self.gridSize)
        row = math.floor((qY - self.minGridY) / self.gridSize)

        return (row, col)

    ##
    # @brief      Plots each of the polygon object grid cells onto ax
    #
    # @param      ax    matplotlib Axes to plot the grid cells on
    # @param      grid  a list of shapely.Polygon objects representing each
    #                   gridcell
    #
    def plotGrid(self, ax, fig, grid):

        newDist = np.flipud(self.distanceCells)
        x, y = np.fliplr(self.numericGridCells)

        c = ax.pcolor(x, y, newDist,
                      edgecolors='k', linewidths=2,
                      cmap='hot', alpha=0.6)
        cbar = fig.colorbar(c, ax=ax)
        cbar.ax.set_ylabel('manhattan distance from obstacle')

    ##
    # @brief      Plot all CSpace objects and saves to self.baseSaveFName
    #
    #             plots obstacles, the robot's path, the start location, and
    #             the goal location
    #
    # @param      robot       A Robot subclass object instance
    # @param      startState  A list with the robot's start coordinates in the
    #                         CSpace coordinate system
    # @param      goalState   A list with the robot's goal state coordinates in
    #                         the CSpace coordinate system
    # @param      plotTitle   The plot title string
    #
    # @return     a plot of the CSpace in the self.baseSaveFName directory
    #
    def plot(self, robot, startState, goalState, plotTitle):

        fig = plt.figure()
        ax = fig.add_subplot(111)

        # plot grid lines BEHIND the fucking data
        ax.set_axisbelow(True)
        self.plotGrid(ax, fig, self.polygonGridCells)

        # plotting all the obstacles
        self.plotObstacles(ax)

        # plotting the robot's motion
        if robot is not None:

            robotPath = robot.stateHistory

            # plotting the robot origin's path through workspace
            x = [state[0] for state in robotPath]
            y = [state[1] for state in robotPath]
            plt.plot(x, y, color='blue', linestyle='solid',
                     linewidth=4, markersize=16)

        # plotting the start / end location of the robot
        plt.plot(startState[0], startState[1],
                 color='green', marker='o', linestyle='solid',
                 linewidth=2, markersize=16)

        plt.plot(goalState[0], goalState[1],
                 color='blue', marker='x', linestyle='solid',
                 linewidth=4, markersize=16)

        # ax.set_axis_off()
        ax.set_aspect('equal')
        plt.title(plotTitle)
        ax.set_xlim(self.minGridX, self.maxGridX)
        ax.set_ylim(self.minGridY, self.maxGridY)

        if self.shouldSavePlots:
            saveFName = self.baseSaveFName + '-' + plotTitle + '.png'
            fig = plt.gcf()
            fig.canvas.manager.full_screen_toggle()
            fig.show()
            fig.set_size_inches((11, 8.5), forward=False)
            plt.savefig(saveFName, dpi=500)
            print('wrote figure to ', saveFName)


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
