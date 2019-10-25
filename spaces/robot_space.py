# 3rd-party packages
import matplotlib.pyplot as plt


# Generic class to implement a spatial representation of a robot or its
# environment for robots with 2-D workspaces
#
class RobotSpace:

    #
    # @brief      RobotSpace class constructor
    #
    # @param      self             The RobotSpace object
    # @param      configData       Configuration dictionary for the RobotSpace
    #                              containing the obstacle coords in the
    #                              space coordinate system
    # @param      shouldSavePlots  Boolean controlling whether or not the plt
    #                              objs can be saved to the baseSaveName dir
    # @param      baseSaveFName    The base directory file name for output plot
    #
    # @return     initialized RobotSpace object
    #
    def __init__(self, configData, shouldSavePlots, baseSaveFName):

        obstacles = self.getObstacles(configData)
        self.obstacles = obstacles

        self.shouldSavePlots = shouldSavePlots
        self.baseSaveFName = baseSaveFName


    #
    # @brief      An abstract function to compute obstacles in the RobotSpace
    #
    # @param      configData  configuration data dictionary for the RobotSpace
    #
    # @return     a list of obstacle coordinates for each obstacle in the
    #             space's coordinate system
    #
    def getObstacles(self, configData):

        return NotImplementedError


    #
    # @brief      Plot all workspace objects and saves to self.baseSaveFName
    #
    #             plots obstacles, the robot's path, the start location, and
    #             the goal location
    #
    # @param      self        The RobotSpace object
    # @param      robotPath   A list with spatial coordinates of the robot's
    #                         path in the space's coordinate system
    # @param      startState  A list with the robot's start coordinates in the
    #                         space's coordinate system
    # @param      goalState   A list with the robot's goal state coordinates in
    #                         the space's coordinate system
    # @param      plotTitle   The plot title string
    #
    # @return     a plot of the RobotSpace in the self.baseSaveFName directory
    #
    def plot(self, robotPath, startState, goalState, plotTitle):

        fig = plt.figure()
        ax = fig.add_subplot(111)

        # plotting all the obstacles
        for obstacle in self.obstacles:
            obstacle.plot()

        # plotting the robot's path
        x, y = zip(*robotPath)
        plt.plot(x, y, color='blue', linestyle='dashed',
                 linewidth=4, markersize=16)

        # plotting the start / end location of the robot
        plt.plot(startState[0], startState[1],
                 color='green', marker='o', linestyle='solid',
                 linewidth=2, markersize=16)

        plt.plot(goalState[0], goalState[1],
                 color='red', marker='x', linestyle='solid',
                 linewidth=4, markersize=16)

        ax.set_axis_off()

        if self.shouldSavePlots:
            saveFName = self.baseSaveFName + '-' + plotTitle + '.png'
            fig = plt.gcf()
            fig.canvas.manager.full_screen_toggle()
            fig.show()
            fig.set_size_inches((11, 8.5), forward=False)
            plt.savefig(saveFName, dpi=500)
            print('wrote figure to ', saveFName)
