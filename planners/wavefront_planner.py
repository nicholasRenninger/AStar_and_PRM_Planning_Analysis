# 3rd-party packages
import copy
from collections import deque
import matplotlib.pyplot as plt
import numpy as np
from timeit import default_timer as timer

# local packages
from planners.planner import Planner
from factory.builder import Builder


##
# @brief      This class describes a Planner subclass used to find a path in
#             two-dimensional configuration space using the wavefront algorithm
#
class WavefrontPlanner(Planner):

    ##
    # @brief      WavefrontPlanner class constructor
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

        # these are the cells from the robot's cspace brushfire routine,
        # before any distance computation
        self.distCells = copy.deepcopy(self.cSpace.initDistanceCells)

    ##
    # @brief      Computes a viable path in robot's cSpace to the goalState
    #             from the robot's startState
    #
    # @param      startState      The start config state
    # @param      goalState       The goal config state
    # @param      plotConfigData  The plot config dictionary
    #
    # @return     a viable set of cSpace states from startState to goalState
    #
    def findPathToGoal(self, startState, goalState, plotConfigData):

        start = timer()
        (distCells,
         foundPath) = self.computeWavefront(startState, goalState,
                                            self.distCells,
                                            self.cSpace.polygonGridCells)
        finish = timer()
        computationTime = finish - start

        # plot the resulting path over the wavefront computation
        plotConfigData['plotTitle'] += 'wavefront'
        self.plot(distCells, startState, goalState, plotConfigData)

        return (computationTime, foundPath)

    ##
    # @brief      Implements the wavefront path finding algorithm
    #
    # @param      startState        The start config state
    # @param      goalState         The goal config state
    # @param      distCells         The uninitialized distance cells
    # @param      polygonGridCells  The polygon grid cells
    #
    # @return     (each cell in distCells is labeled with its mimum manhattan
    #             distance from an obstacle, boolean indicating whether or not
    #             a path was found)
    #
    def computeWavefront(self, startState, goalState, distCells,
                         polygonGridCells):

        N = self.cSpace.N

        (neighbors,
         distCells,
         (goalRow, goalCol),
         goalCell) = self.initializeWavefront(goalState, polygonGridCells,
                                              distCells, N)

        # need to keep a dictionary of neighbors so we can look them up in
        # the future
        allNeighbors = {}
        allNeighbors[(goalRow, goalCol)] = goalCell

        (startRow, startCol) = self.cSpace.getGridCoordsFromState(startState)

        foundPath = ((startRow == goalRow) and (startCol == goalCol))
        while neighbors:

            # using a the deque object as a queue to maintain "wavefront"
            neighbor = neighbors.popleft()

            # update distance for all current neighbors
            neighborRow = neighbor[0]
            neighborCol = neighbor[1]
            prevDist = neighbor[2]
            currDist = prevDist + 1
            distCells[neighborRow, neighborCol] = currDist

            # break once you hit the start state - you've found a path
            foundPath = ((startRow == neighborRow) and
                         (startCol == neighborCol))

            if foundPath:

                foundPath = True
                break

            # store the neighbor for backtracking
            prevRow, prevCol = neighbor[3]
            allNeighbors[(neighborRow, neighborCol)] = neighbor

            # now find all possible new neighbors from the current cell
            possibleNewNeighbors = self.cSpace.calcNeighbors(neighborRow,
                                                             neighborCol,
                                                             N,
                                                             dist=currDist)

            # add unvisited, unique neighbors to search
            neighbors = self.cSpace.updateNeighborCells(distCells,
                                                        possibleNewNeighbors,
                                                        neighbors)

        # turn all zeros (parts of the map for which we have no path) to nans
        # so the plot just doesn't show anything
        distCells = np.where(distCells == 0, np.NaN, distCells)

        # searched all reachable neighbors, but never found a path to start
        if not foundPath:

            print('failed to reach start startState with wavefront.')
            print('Reached cell id: (', neighborRow, ',', neighborCol, ')')
            foundPath = False

        else:

            # now backtrack through the solution from the start
            atGoal = ((startRow == goalRow) and (startCol == goalCol))
            currentCell = neighbor
            while not atGoal:

                currRow = currentCell[0]
                currCol = currentCell[1]

                currState = self.cSpace.getStateFromGridIndices(currRow,
                                                                currCol)
                self.robot.updateRobotState(currState)

                # check for goal, as goal has no previous coords
                atGoal = currentCell[3] is None
                if atGoal:

                    self.robot.updateRobotState(goalState)
                    break

                else:

                    prevCellIndices = currentCell[3]
                    currentCell = allNeighbors[prevCellIndices]

        return (distCells, foundPath)

    ##
    # @brief      Initializes the wavefront algorithm with a distanceCell
    #             object and a set of neighbors in a queue to begin searching
    #
    # @param      goalState         The goal config state
    # @param      polygonGridCells  a list of polygon objects - one for each
    #                               grid cell
    # @param      distCells         a 0s 2D numpy array corresponding to the
    #                               CSpace grid
    # @param      N                 linear discretization density of CSpace
    #
    # @return     (a unique list of indices of neighbors of the goal cell, a 2D
    #             numpy array with a 1 at all cells intersecting an obstacle
    #             and 0s everywhere else, a tuple containing the grid indices
    #             of the goal state, a "neighbors" element at the goal for
    #             backtracking)
    #
    def initializeWavefront(self, goalState, polygonGridCells, distCells, N):

        # start by computing the set of obstacle cells and their neighbors
        (_,
         distCells,
         _) = self.cSpace.labelObstacleCells(polygonGridCells, distCells, N)

        # start the brushfire algorithm from the goal cell with a distance of 2
        currDist = 2
        (goalRow, goalCol) = self.cSpace.getGridCoordsFromState(goalState)
        distCells[goalRow, goalCol] = currDist

        # need to initialize a "neighbors" element at the goalState for
        # backtracking
        goalCell = []
        goalCell.append(goalRow)
        goalCell.append(goalCol)
        goalCell.append(currDist)

        # goal has no parent cell
        goalCell.append(None)

        # add all neighbors of the goal state to the neighbor search queue
        possibleNeighbors = self.cSpace.calcNeighbors(goalRow, goalCol,
                                                      N, currDist)

        neighborsOfGoal = deque()
        neighborsOfGoal = self.cSpace.updateNeighborCells(distCells,
                                                          possibleNeighbors,
                                                          neighborsOfGoal)
        neighbors = copy.deepcopy(neighborsOfGoal)

        return neighbors, distCells, (goalRow, goalCol), goalCell

    ##
    # @brief      plots the wavefront distance computation overlaid on the
    #             cspace plot
    #
    # @param      distCells       The distance cells with wavefront distance
    #                             computed
    # @param      startState      The start config state
    # @param      goalState       The goal config state
    # @param      plotConfigData  The plot configuration data
    #
    def plot(self, distCells, startState, goalState, plotConfigData):

        fig = plt.figure()
        ax = fig.add_subplot(111)

        cBarLabel = 'manhattan distance from goal'
        x, y = self.cSpace.numericGridCells
        self.cSpace.plotGrid(ax, fig, self.cSpace.polygonGridCells, distCells,
                             x, y, cBarLabel, colormap='viridis')

        self.cSpace.plot(robot=self.robot,
                         startState=startState,
                         goalState=goalState,
                         plotConfigData=plotConfigData,
                         ax=ax, fig=fig)


##
# @brief      Implements the generic builder class for the WavefrontPlanner
#
class WavefrontPlannerBuilder(Builder):

    ##
    # @brief      need to call the super class constructor to gain its
    #             properties
    #
    def __init__(self):

        Builder.__init__(self)
        self.robot = None
        self.workspace = None

    ##
    # @brief      Implements the smart constructor for WavefrontPlanner
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
    # @return     instance of an initialized WavefrontPlanner object
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
            self._instance = WavefrontPlanner(plannerType=plannerType,
                                              cSpace=cSpace,
                                              workspace=workspace,
                                              robot=robot,
                                              configData=configData,
                                              shouldSavePlots=shouldSavePlots,
                                              baseSaveFName=baseSaveFName)

        return self._instance
