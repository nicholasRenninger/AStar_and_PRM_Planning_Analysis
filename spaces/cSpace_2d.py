# 3rd-party packages
import matplotlib.pyplot as plt
import numpy as np
from shapely.geometry import Polygon
import copy
from collections import deque as deque
from descartes.patch import PolygonPatch
import math

# local packages
from spaces.robot_space import RobotSpace


##
# @brief      Concrete implementation of the 2D cspace for a point robot
#
class CSpace_2D(RobotSpace):

    ##
    # @brief      CSpace_2D class constructor
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
    # @return     initialized CSpace_2D object
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
        self.minGridX = None
        self.maxGridX = None
        self.minGridY = None
        self.maxGridY = None

        self.linearDiscretizationDensity = linearDiscretizationDensity

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

        for _, gridCell in grid.items():
                patch = PolygonPatch(gridCell,
                                     edgecolor='#6699cc',
                                     facecolor='none',
                                     alpha=0.8)
                ax.add_patch(patch)

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
    #
    # @return     matplotlib Axes object for the generated plot
    #
    def plot(self, robot, startState, goalState, plotConfigData):

        # unpack dictionary
        plotTitle = plotConfigData['plotTitle']
        xlabel = plotConfigData['xlabel']
        ylabel = plotConfigData['ylabel']

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

            # plotting the robot origin's path through cspace
            x = [state[0] for state in robotPath]
            y = [state[1] for state in robotPath]
            plt.plot(x, y, color='blue', linestyle='solid',
                     linewidth=4, markersize=16,
                     label='Robot path')

        # plotting the start / end location of the robot
        plt.plot(startState[0], startState[1],
                 color='green', marker='o', linestyle='none',
                 linewidth=2, markersize=16,
                 label='Starting State')

        plt.plot(goalState[0], goalState[1],
                 color='blue', marker='x', linestyle='none',
                 linewidth=4, markersize=16,
                 label='Goal State')

        # ax.set_axis_off()
        ax.set_aspect('equal')
        plt.title(plotTitle)
        plt.xlabel(xlabel)
        plt.ylabel(ylabel)
        ax.set_xlim(self.minGridX, self.maxGridX)
        ax.set_ylim(self.minGridY, self.maxGridY)
        ax.legend()

        if self.shouldSavePlots:
            saveFName = self.baseSaveFName + '-' + plotTitle + '.png'
            fig = plt.gcf()
            fig.canvas.manager.full_screen_toggle()
            fig.show()
            fig.set_size_inches((11, 8.5), forward=False)
            plt.savefig(saveFName, dpi=500)
            print('wrote figure to ', saveFName)

        return ax
