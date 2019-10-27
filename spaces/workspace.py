# 3rd-party packages
import matplotlib.pyplot as plt
import numpy as np
from shapely.geometry import Polygon

# local packages
from spaces.robot_space import RobotSpace
from factory.builder import Builder


##
# @brief      Class for 2D robot Workspace objects
#
# @warning    Workspace can only be comprised of polygonal obstacles
class Workspace(RobotSpace):

    ##
    # @brief      Workspace class constructor
    #
    # @param      self             The Workspace object object
    # @param      configData       Configuration dictionary for the robot
    #                              containing the obstacle coords
    # @param      shouldSavePlots  Boolean controlling whether or not the plt
    #                              objs can be saved to the baseSaveName dir
    # @param      baseSaveFName    The base directory file name for output plot
    #
    # @return     initialized Workspace object
    #
    def __init__(self, configData, shouldSavePlots, baseSaveFName):

        # get inherited properties from superclass implementation
        RobotSpace.__init__(self,
                            shouldSavePlots=shouldSavePlots,
                            baseSaveFName=baseSaveFName)

        # initialize the obstacle bounding box nicely for later ;)
        self.minXObst = np.inf
        self.maxXObst = -np.inf
        self.minYObst = np.inf
        self.maxYObst = -np.inf

        (obstacles, polygonObstacles) = self.getObstacles(configData)
        self.obstacles = obstacles
        self.polygonObstacles = polygonObstacles

    ##
    # @brief      Creates a list of obstacles from config data
    #
    # @param      configData  configuration data dictionary for the robot
    #
    # @return     (a list of Workspace obstacle vertex coordinates for each
    #             obstacle,
    #
    #             a list of shapely.geometry.Polygon objects corresponding to
    #             each set of vertices in the workspace)
    #
    def getObstacles(self, configData):

        obstacles = configData['WO']

        polygonObstacles = []
        for obstacle in obstacles:
            currObstacle = Polygon(obstacle)
            polygonObstacles.append(currObstacle)

            # we need to compute a bounding box in workspace for all of the
            # obstacles for planning algorithms
            (minx, miny, maxx, maxy) = currObstacle.bounds
            self.minXObst = min(minx, self.minXObst)
            self.maxXObst = max(maxx, self.maxXObst)
            self.minYObst = min(miny, self.minYObst)
            self.maxYObst = max(maxy, self.maxYObst)

        return (np.array(obstacles, dtype='float64'), polygonObstacles)

    ##
    # @brief      Plots all obstacles in the workspace to ax
    #
    # @param      ax    the matplotlib.axes object to plot the obstacles on
    #
    def plotObstacles(self, ax):

        for obstacle in self.obstacles:
            obstX, obstY = zip(*obstacle)
            ax.fill(obstX, obstY,
                    facecolor='black',
                    edgecolor='black',
                    linewidth=3)

    ##
    # @brief      Plot all Workspace objects and saves to self.baseSaveFName
    #
    #             plots obstacles, the robot's path, the start location, and
    #             the goal location
    #
    # @param      robot       A Robot subclass object instance
    # @param      startState  A list with the robot's start coordinates in the
    #                         Workspace coordinate system
    # @param      goalState   A list with the robot's goal state coordinates in
    #                         the Workspace coordinate system
    # @param      plotTitle   The plot title string
    #
    # @return     a plot of the Workspace in the self.baseSaveFName directory
    #
    def plot(self, robot, startState, goalState, plotTitle):

        fig = plt.figure()
        ax = fig.add_subplot(111)

        # plot grid lines BEHIND the fucking data
        ax.set_axisbelow(True)
        plt.grid()

        # plotting all the obstacles
        self.plotObstacles(ax)

        # plotting the robot's motion
        if robot is not None:

            robotPath = robot.stateHistory

            robot.plotBodyInWorkspace(ax)

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
                 color='red', marker='x', linestyle='solid',
                 linewidth=4, markersize=16)

        # ax.set_axis_off()
        ax.set_aspect('equal')
        plt.title(plotTitle)

        if self.shouldSavePlots:
            saveFName = self.baseSaveFName + '-' + plotTitle + '.png'
            fig = plt.gcf()
            fig.canvas.manager.full_screen_toggle()
            fig.show()
            fig.set_size_inches((11, 8.5), forward=False)
            plt.savefig(saveFName, dpi=500)
            print('wrote figure to ', saveFName)


##
# @brief      Implements the generic builder class for the Workspace
#
class WorkspaceBuilder(Builder):

    # need to call the super class constructor to gain its properties
    #
    def __init__(self):
        Builder.__init__(self)

    ##
    # @brief      Implements the smart constructor for Workspace
    #
    # Only reads the config data once, otherwise just returns the built object
    #
    # @param      configFileName   The YAML configuration file name
    # @param      shouldSavePlots  The should save plots
    # @param      baseSaveFName    The base directory file name for output plot
    #
    def __call__(self, configFileName, shouldSavePlots, baseSaveFName):

        configNameHasChanged = (self._configName != configFileName)
        noInstanceLoadedYet = (self._instance is None)

        if noInstanceLoadedYet or configNameHasChanged:

            self._configName = configFileName
            configData = self.loadConfigData(configFileName)
            self._instance = Workspace(configData=configData,
                                       shouldSavePlots=shouldSavePlots,
                                       baseSaveFName=baseSaveFName)

        return self._instance
