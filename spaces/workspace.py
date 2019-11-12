# local packages
from spaces.robot_space import RobotSpace
from factory.builder import Builder
from util.plots import savePlot

# 3rd-party packages
import numpy as np
from shapely.geometry import Polygon
import imageio
import matplotlib
matplotlib.use('TkAgg')
import matplotlib.pyplot as plt


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
        plt.grid()

        # plotting all the obstacles
        self.plotObstacles(ax)

        # plotting the start / end location of the robot
        plt.plot(startState[0], startState[1],
                 color='green', marker='o', linestyle='none',
                 linewidth=2, markersize=16,
                 label='Starting State')

        plt.plot(goalState[0], goalState[1],
                 color='red', marker='x', linestyle='none',
                 linewidth=4, markersize=16,
                 label='Goal State')

        # plotting the robot's motion
        if robot is not None:

            robotPath = robot.stateHistory

            images = robot.plotBodyInWorkspace(ax, fig)

            # plotting the robot origin's path through workspace
            x = [state[0] for state in robotPath]
            y = [state[1] for state in robotPath]
            plt.plot(x, y, color='red', linestyle='solid',
                     linewidth=4, markersize=16,
                     label='Robot path')

        ax.set_aspect('equal')
        plt.title(plotTitle)
        fig.legend(loc='upper left')
        plt.xlabel(xlabel)
        plt.ylabel(ylabel)

        savePlot(fig=fig, shouldSavePlots=self.shouldSavePlots,
                 baseSaveFName=self.baseSaveFName, plotTitle=plotTitle)

        # saving the animation data
        if self.shouldSavePlots and (images is not None):

            gifSaveFName = self.baseSaveFName + '-' + plotTitle + '.gif'
            imageio.mimsave(gifSaveFName, images, fps=20)
            print('wrote figure to: ', gifSaveFName)

        return ax


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
