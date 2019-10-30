# 3rd-party packages
import numpy as np

# local packages
from factory.builder import Builder
from spaces.cSpace_2d import CSpace_2D


##
# @brief      Concrete implementation of the 2D cspace for a point robot
#
class ManipulatorRobotCSpace(CSpace_2D):

    ##
    # @brief      ManipulatorRobotCSpace class constructor
    #
    # @param      robot            The ManipulatorRobot instance
    # @param      workspace        The CSpace object the ManipulatorRobot
    #                              operates in
    # @param      N                linear discretization density of CSpace
    # @param      shouldSavePlots  Boolean controlling whether or not the plt
    #                              objs can be saved to the baseSaveName dir
    # @param      baseSaveFName    The base directory file name for output plot
    #
    # @return     initialized ManipulatorRobotCSpace object
    #
    def __init__(self, robot, workspace, N,
                 shouldSavePlots, baseSaveFName):

        # get inherited properties from superclass implementation
        CSpace_2D.__init__(self, robot, workspace, N,
                           shouldSavePlots, baseSaveFName)

        # need a bounding box for the discretization of CSpace
        (self.minGridX,
         self.maxGridX,
         self.minGridY,
         self.maxGridY) = self.determineSpatialBounds()

        # need to discretize the cspace for the brushfire / wavefront algorithm
        (self.polygonGridCells,
         self.numericGridCells,
         self.numericCoordArrays,
         initDistanceCells,
         (self.xGridSize,
          self.yGridSize)) = self.discretizeCSpace(N)

        # save the initial distance cells to be used by the wavefront planner
        self.initDistanceCells = initDistanceCells

        # need to compute the obstacles on the CSpace grid
        self.obstacles = self.computeCSpaceObstacles()

        print('Built CSpace with grid cells of size :',
              self.xGridSize, ' x ', self.yGridSize)

    ##
    # @brief      Determines a spatial bounding box for CSpace enclosing the
    #             robot's configurations and all of its obstacles
    #
    #             as its a two link manipulator, this is just a 2 pi x 2 pi
    #             grid
    #
    # @return     (minGridX, maxGridX, minGridY, maxGridY)
    #
    def determineSpatialBounds(self):

        # [rad]
        minGridX = 0.0
        maxGridX = 2 * np.pi
        minGridY = 0
        maxGridY = 2 * np.pi

        return (minGridX, maxGridX, minGridY, maxGridY)

    ##
    # @brief      Approximates the c space obstacles from workspace obstacles
    #
    def computeCSpaceObstacles(self):

        rows, cols = self.initDistanceCells.shape

        # represent obstacles as either a 0 (no obstacle) or a 1 (obstacle) on
        # the same grid as the distance computation
        cSpaceObstacles = np.zeros((rows, cols), dtype='int32')

        # check for collision at each location on the grid
        for i in range(0, rows):
            for j in range(0, cols):

                # get the continuous c state so we can test for collison by
                # moving the manipulator there and seeing if there is collision
                currCState = self.getStateFromGridIndices(i, j)

                collided = self.robot.checkCollisionWithState(currCState)

                # mark that the grid cell contained some workspace obstacle
                if collided:
                    cSpaceObstacles[i, j] = 1

        return cSpaceObstacles

    ##
    # @brief      Plots all obstacles in the cspace to ax
    #
    # @param      ax    the matplotlib.axes object to plot the obstacles on
    # @param      fig   matplotlib Figure to plot the grid cells on
    #
    def plotObstacles(self, ax, fig):

        xMesh, yMesh = self.numericGridCells

        c = ax.pcolor(xMesh, yMesh, self.obstacles,
                      edgecolors='k', linewidths=0,
                      cmap='seismic', alpha=0.6)

        cbar = fig.colorbar(c, ax=ax, orientation="vertical")
        cbar.ax.set_ylabel(('boolean obstacle occupany ' +
                            '(0 = freespace, 1 = obstacle)'))


##
# @brief      Implements the generic builder class for the
#             ManipulatorRobotCSpace
#
class ManipulatorRobotCSpaceBuilder(Builder):

    # need to call the super class constructor to gain its properties
    #
    def __init__(self):
        Builder.__init__(self)
        self.robot = None
        self.workspace = None

    ##
    # @brief      Implements the smart constructor for ManipulatorRobotCSpace
    #
    #             Only reads the config data once, otherwise just returns the
    #             built object
    #
    # @param      robot            The ManipulatorRobotCSpace type string
    # @param      N                linear discretization density of CSpace
    # @param      shouldSavePlots  Boolean controlling whether or not the plt
    #                              objs can be saved to the baseSaveName dir
    # @param      baseSaveFName    The base directory file name for output plot
    #
    # @return     instance of an initialized ManipulatorRobotCSpace object
    #
    def __call__(self, robot, N, shouldSavePlots, baseSaveFName):

        robotHasChanged = (self.robot != robot)
        workspaceHasChanged = (self.workspace != robot.workspace)
        noInstanceLoadedYet = (self._instance is None)

        if noInstanceLoadedYet or robotHasChanged or workspaceHasChanged:

            self._instance = \
                ManipulatorRobotCSpace(robot=robot,
                                       workspace=robot.workspace,
                                       N=N,
                                       shouldSavePlots=shouldSavePlots,
                                       baseSaveFName=baseSaveFName)

        return self._instance
