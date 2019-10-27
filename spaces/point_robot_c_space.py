# 3rd-party packages
import matplotlib.pyplot as plt

# local packages
from factory.builder import Builder
from spaces.robot_space import RobotSpace


##
# @brief      Concrete implementation of the cspace for a point robot
#
class PointRobotCSpace(RobotSpace):

    ##
    # @brief      PointRobotCSpace class constructor
    #
    # @param      robot            The PointRobot instance
    # @param      workspace        The CSpace object the PointRobot
    #                              operates in
    # @param      shouldSavePlots  Boolean controlling whether or not the plt
    #                              objs can be saved to the baseSaveName dir
    # @param      baseSaveFName    The base directory file name for output plot
    #
    # @return     initialized PointRobotCSpace object
    #
    def __init__(self, robot, workspace, shouldSavePlots, baseSaveFName):

        # get inherited properties from superclass implementation
        RobotSpace.__init__(self, shouldSavePlots, baseSaveFName)

        self.workspace = robot.workspace
        self.robot = robot

        # as the robot is a point robot, its obstacles are just
        self.obstacles = robot.workspace.obstacles

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
                    linewidth=3)

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
        plt.grid()

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
    # @param      workspace        The CSpace object the PointRobot
    #                              operates in
    # @param      shouldSavePlots  Boolean controlling whether or not the plt
    #                              objs can be saved to the baseSaveName dir
    # @param      baseSaveFName    The base directory file name for output plot
    #
    # @return     instance of an initialized PointRobotCSpace object
    #
    def __call__(self, robot, shouldSavePlots, baseSaveFName):

        robotHasChanged = (self.robot != robot)
        workspaceHasChanged = (self.workspace != robot.workspace)
        noInstanceLoadedYet = (self._instance is None)

        if noInstanceLoadedYet or robotHasChanged or workspaceHasChanged:

            self._instance = \
                PointRobotCSpace(robot=robot,
                                 workspace=robot.workspace,
                                 shouldSavePlots=shouldSavePlots,
                                 baseSaveFName=baseSaveFName)

        return self._instance
