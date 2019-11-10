# 3rd-party packages
import matplotlib.pyplot as plt
import numpy as np
from shapely.geometry import Polygon
import copy
from collections import deque as deque
import math

# local packages
from spaces.robot_space import RobotSpace
from util.plots import savePlot


##
# @brief      Concrete implementation of the 2D cspace for a point robot
#
class CSpace_2D(RobotSpace):

    ##
    # @brief      CSpace_2D class constructor
    #
    # @param      robot            The PointRobot instance
    # @param      workspace        The CSpace object the PointRobot operates in
    # @param      N                linear discretization density of CSpace
    # @param      shouldSavePlots  Boolean controlling whether or not the plt
    #                              objs can be saved to the baseSaveName dir
    # @param      baseSaveFName    The base directory file name for output plot
    #
    # @return     initialized CSpace_2D object
    #
    def __init__(self, robot, workspace, N,
                 shouldSavePlots, baseSaveFName):

        # get inherited properties from superclass implementation
        RobotSpace.__init__(self, shouldSavePlots, baseSaveFName)

        self.workspace = robot.workspace
        self.robot = robot

        self.cStateHistory = []

        self.obstacles = None
        self.polygonObstacles = None

        # need a bounding box for the discretization of CSpace
        self.minGridX = None
        self.maxGridX = None
        self.minGridY = None
        self.maxGridY = None

        self.N = N

    ##
    # @brief      discretizes CSpace into an NxN grid, based on the bounding
    #             box for CSpace
    #
    # @param      N     number of grid points in each CSpace dimension
    #
    # @return     (a list of polygon objects - one for each grid cell, a numpy
    #             meshgrid representing the grid coordinates, a tuple with the
    #             numpy array of the raw grid coordinates of both the x and y
    #             axes, a 2D numpy array of zeros used in the brushfire
    #             algorithms, the size of each grid cell, a tuple of the x and
    #             y sizes of the grids)
    #
    def discretizeCSpace(self, N):

        xCoords = np.linspace(self.minGridX, self.maxGridX, N + 1)
        yCoords = np.linspace(self.minGridY, self.maxGridY, N + 1)

        xGridSize = abs(xCoords[1] - xCoords[0])
        yGridSize = abs(yCoords[1] - yCoords[0])

        xMesh, yMesh = np.meshgrid(xCoords, yCoords)
        emptyDistanceCells = np.zeros((N, N))

        # need to store the grid as an indexed set of cells for brushfire need
        # to use a dictionary, lists of lists are reference traps
        #
        # Here, we want the i index to
        # refer to the row of the cell, and j to correspond to the column of
        # the cell
        #
        # j = 0, i = 0: bottom left corner of grid
        polygonGridCells = {}
        for i in range(N):
            for j in range(N):

                cell = self.getCSpaceStatesOfGridVertices(i, j, xMesh, yMesh)

                cellPolygon = Polygon(cell)
                polygonGridCells[(i, j)] = copy.deepcopy(cellPolygon)

        return (polygonGridCells, (xMesh, yMesh), (xCoords, yCoords),
                emptyDistanceCells, (xGridSize, yGridSize))

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
                obstacleInCell = self.robot.checkCollision(currCell, (i, j))

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
    #             Stores the distance to its "parent" cell, and the parent
    #             cell's indices themselves
    #
    #             manipulator's cspace is a torus, so it wraps around the
    #             bounds
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
            neighbors = [(i + 1, j, dist, (i, j)), (i, j + 1, dist, (i, j))]

            if self.robot.robotType == 'MANIPULATOR':
                neighbors.extend([(i - 1, j, dist, (i, j)),
                                  (i, j - 1, dist, (i, j))])

        elif i == 0 and j == N - 1:
            # bottom right corner
            neighbors = [(i + 1, j, dist, (i, j)), (i, j - 1, dist, (i, j))]

            if self.robot.robotType == 'MANIPULATOR':
                neighbors.extend([(i - 1, j, dist, (i, j)),
                                  (i, 0, dist, (i, j))])

        elif i == N - 1 and j == 0:
            # top left corner
            neighbors = [(i - 1, j, dist, (i, j)), (i, j + 1, dist, (i, j))]

            if self.robot.robotType == 'MANIPULATOR':
                neighbors.extend([(0, j, dist, (i, j)),
                                  (i, j - 1, dist, (i, j))])

        elif i == N - 1 and j == N - 1:
            # top right corner
            neighbors = [(i - 1, j, dist, (i, j)), (i, j - 1, dist, (i, j))]

            if self.robot.robotType == 'MANIPULATOR':
                neighbors.extend([(0, j, dist, (i, j)),
                                  (i, 0, dist, (i, j))])

        elif i == 0:
            # bottom edge
            neighbors = [(i, j + 1, dist, (i, j)), (i, j - 1, dist, (i, j)),
                         (i + 1, j, dist, (i, j))]

            if self.robot.robotType == 'MANIPULATOR':
                neighbors.append((i - 1, j, dist, (i, j)))

        elif i == N - 1:
            # top edge
            neighbors = [(i, j + 1, dist, (i, j)), (i, j - 1, dist, (i, j)),
                         (i - 1, j, dist, (i, j))]

            if self.robot.robotType == 'MANIPULATOR':
                neighbors.append((0, j, dist, (i, j)))

        elif j == 0:
            # left edge
            neighbors = [(i + 1, j, dist, (i, j)), (i - 1, j, dist, (i, j)),
                         (i, j + 1, dist, (i, j))]

            if self.robot.robotType == 'MANIPULATOR':
                neighbors.append((i, j - 1, dist, (i, j)))

        elif j == N - 1:
            # right edge
            neighbors = [(i + 1, j, dist, (i, j)), (i - 1, j, dist, (i, j)),
                         (i, j - 1, dist, (i, j))]

            if self.robot.robotType == 'MANIPULATOR':
                neighbors.append((i, 0, dist, (i, j)))

        else:
            # center
            neighbors = [(i + 1, j, dist, (i, j)), (i - 1, j, dist, (i, j)),
                         (i, j + 1, dist, (i, j)), (i, j - 1, dist, (i, j))]

        return neighbors

    ##
    # @brief      Implements the brushfire algorithm
    #
    # @param      distCells         The 2D distance cells holding distance
    #                               from obstacles at each cell
    # @param      polygonGridCells  The polygon grid cells
    #
    # @return     (each cell in distCells is labeled with its mimum manhattan
    #              distance from an obstacle,
    #              maximum distance from an obstacle)
    #
    def brushFireDistanceComputation(self, distCells,
                                     polygonGridCells):

        N = self.N

        # start by computing the set of obstacle cells and their neighbors
        (neighborsOfCellsWithObstacles,
         distCells,
         _) = self.labelObstacleCells(polygonGridCells, distCells, N)
        currDist = 1

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
    # @brief      updates newNeighbors with a list of unvisited, unique
    #             neighbors with the list of given possible new neighbors
    #
    # @param      distanceCells         The distanceCells to update the
    #                                   distance measure in
    # @param      possibleNewNeighbors  The possible new neighbors index list
    # @param      newNeighbors          The neighboring cell index list to add
    #                                   valid neighbors to
    #
    # @return     newNeighbors has unvisited, unique neighbor cells appended
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
    # @brief      Gets the grid coordinates in the discretized cspace from
    #             state
    #
    # @param      state  The queried state coordinate
    #
    # @return     (row of self.distanceCells corresponding to the state, column
    #             of self.distanceCells corresponding to the state)
    #
    def getGridCoordsFromState(self, state):

        # need to convert numpy array for state into list of coordinates
        # stateCoords = np.concatenate(state, axis=0)
        stateCoords = state
        qX = stateCoords[0]
        qY = stateCoords[1]

        # these produce wrong indices when the point is right on a maxima of
        # the grid
        gridShrinkScale = 10
        if qX == self.maxGridX:
            qX -= self.xGridSize / gridShrinkScale

        if qY == self.maxGridY:
            qY -= self.yGridSize / gridShrinkScale

        col = math.floor((qX - self.minGridX) / self.xGridSize)
        row = math.floor((qY - self.minGridY) / self.yGridSize)

        return (row, col)

    ##
    # @brief      Gets the cspace states of grid vertices given the grid
    #             coordinates (indices)
    #
    # @param      row    The row index of the grid
    # @param      col    The col index of the grid
    # @param      xMesh  2D numpy array of the x grid mesh
    # @param      yMesh  2D numpy array of the y grid mesh
    #
    # @return     a list of cspace states of grid vertices at the row / col
    #             index
    #
    def getCSpaceStatesOfGridVertices(self, row, col, xMesh, yMesh):

        # yuck :(
        statesOfVertices = [(xMesh[row][col], yMesh[row][col]),
                            (xMesh[row][col + 1], yMesh[row][col + 1]),
                            (xMesh[row + 1][col + 1], yMesh[row + 1][col + 1]),
                            (xMesh[row + 1][col], yMesh[row + 1][col])]

        return statesOfVertices

    ##
    # @brief      returns the cpsace centroid coordinates of the grid cell at
    #             the given grid indices
    #
    # @param      rowIdx  The cspace grid row index
    # @param      colIdx  The cspace grid col index
    #
    # @return     The cspace coordinates of the centroid of the grid at indices
    #
    def getStateFromGridIndices(self, rowIdx, colIdx):

        xMesh, yMesh = self.numericGridCells

        cell = self.getCSpaceStatesOfGridVertices(rowIdx, colIdx, xMesh, yMesh)

        topLeft = cell[0]
        bottomRight = cell[3]

        x_tl, y_tl = topLeft
        x_br, y_br = bottomRight

        xCentroid = (x_tl + x_br) / 2.0
        yCentroid = (y_tl + y_br) / 2.0
        centroid = [xCentroid, yCentroid]

        # give it in the same format as other states
        return np.array(centroid, dtype='float64').reshape((2, 1))

    ##
    # @brief      Abstract function to plot all obstacles in the cspace to ax
    #
    # @param      ax    the matplotlib.axes object to plot the obstacles on
    # @param      fig   The fig
    #
    def plotObstacles(self, ax, fig):

        pass

    ##
    # @brief      Plots each of the polygon object grid cells onto ax
    #
    # @warning    overrides method in superclass
    #
    # @param      ax             matplotlib Axes to plot the grid cells on
    # @param      fig            matplotlib Figure to plot the grid cells on
    # @param      grid           a list of shapely.Polygon objects representing
    #                            each gridcell
    # @param      distanceCells  The 2D distance cells holding distance from
    #                            obstacles at each cell
    # @param      x              numpy array of the raw x grid coordinates
    # @param      y              numpy array of the raw y grid coordinates
    # @param      cBarLabel      The color bar label string
    # @param      colormap       The colormap string
    #
    def plotGrid(self, ax, fig, grid, distanceCells, x, y,
                 cBarLabel, colormap):

        c = ax.pcolor(x, y, distanceCells,
                      edgecolors='k', linewidths=0,
                      cmap=colormap, alpha=0.6)

        cbar = fig.colorbar(c, ax=ax, orientation="vertical")
        cbar.ax.set_ylabel(cBarLabel)

    ##
    # @brief      Plot all CSpace objects and saves to self.baseSaveFName
    #
    #             plots obstacles, the robot's path, the start location, and
    #             the goal location
    #
    #             Plots are written to the self.baseSaveFName directory
    #
    # @param      robot           A Robot subclass object instance
    # @param      startState      A list with the robot's start coordinates in
    #                             the CSpace coordinate system
    # @param      goalState       A list with the robot's goal state
    #                             coordinates in the CSpace coordinate system
    # @param      plotConfigData  The plot configuration data dictionary:
    #                             - plotTitle  The plot title string
    #                             - xlabel     xlabel string
    #                             - ylabel     ylabel string
    #                             - plotGrid   bool to turn on/off the grid
    # @param      fig             matplotlib Figure to plot the grid cells on
    # @param      ax              matplotlib Axes corresponding to fig to plot
    #                             the data on
    #
    # @return     matplotlib Axes object for the generated plot
    #
    def plot(self, robot, startState, goalState, plotConfigData,
             fig=None, ax=None):

        # unpack dictionary
        plotTitle = plotConfigData['plotTitle']
        xlabel = plotConfigData['xlabel']
        ylabel = plotConfigData['ylabel']
        shouldPlotCSpaceDiscretizationGrid = plotConfigData['plotGrid']
        shouldPlotObstacles = plotConfigData['plotObstacles']

        if not ax or not fig:
            fig = plt.figure()
            ax = fig.add_subplot(111)

        # plot grid lines BEHIND the data
        ax.set_axisbelow(True)

        if shouldPlotCSpaceDiscretizationGrid:
            cBarLabel = 'manhattan distance from obstacle'
            distCells = self.distanceCells
            x, y = self.numericGridCells
            self.plotGrid(ax, fig, self.polygonGridCells, distCells,
                          x, y, cBarLabel, colormap='hot')
        else:
            plt.grid()

        # plotting all the obstacles
        if shouldPlotObstacles:
            self.plotObstacles(ax, fig)

        # plotting the robot's motion
        if robot is not None:

            robotPath = robot.cStateHistory

            # plotting the robot origin's path through cspace
            x = [state[0] for state in robotPath]
            y = [state[1] for state in robotPath]
            plt.plot(x, y, color='red', marker='*', linestyle='none',
                     linewidth=4, markersize=3,
                     label='Robot path')

        # plotting the start / end location of the robot
        plt.plot(startState[0], startState[1],
                 color='green', marker='o', linestyle='none',
                 linewidth=2, markersize=16,
                 label='Starting State')

        plt.plot(goalState[0], goalState[1],
                 color='red', marker='x', linestyle='none',
                 linewidth=4, markersize=16,
                 label='Goal State')

        ax.set_aspect('equal')
        plt.title(plotTitle)
        plt.xlabel(xlabel)
        plt.ylabel(ylabel)
        ax.set_xlim(self.minGridX, self.maxGridX)
        ax.set_ylim(self.minGridY, self.maxGridY)
        fig.legend(loc='upper left')

        savePlot(fig=fig, shouldSavePlots=self.shouldSavePlots,
                 baseSaveFName=self.baseSaveFName, plotTitle=plotTitle)

        return ax
