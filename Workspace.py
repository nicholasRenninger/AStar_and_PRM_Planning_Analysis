import yaml
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle


# @brief      Class for 2D robot workspace objects
#
#  Workspace can only be comprised of rectangular obstacles, a
#  start location, and a goal location
class Workspace:

    #
    # @brief      Workspace class constructor
    #
    # @param      self            The workspace object object
    # @param      configFileName  The workspace configuration file name
    #                             contains the obstacle coords
    #
    # @return     initialized workspace object
    #
    def __init__(self, configFileName, shouldSavePlots, baseSaveFName):

        obstacles = self.getEnvFromFile(configFileName)
        self.obstacles = obstacles

        self.shouldSavePlots = shouldSavePlots
        self.baseSaveFName = baseSaveFName

    #
    # @brief      Creates a list of obstacles from a YAML config file
    #
    # @return     a list of workspace obstacle vertex coordinates for each
    #             obstacle
    #
    @staticmethod
    def getEnvFromFile(configFileName):

        with open(configFileName, 'r') as stream:
            config_data = yaml.safe_load(stream)

        obstacles = config_data['WO']
        return obstacles


    def 

    #
    # @brief      Plot all workspace objects and saves to self.baseSaveFName
    #
    #             obstacles, the robot's path, the start location, and the goal
    #             location
    #
    # @param      self        The workspace object
    # @param      robotPath   A list with workspace coordinates of the robot's
    #                         path
    # @param      startState  A list with the robot's start state coordinates
    # @param      goalState   A list with the robot's goal state coordinates
    # @param      plotTitle   The plot title string
    #
    # @return     a plot of the workspace in the self.baseSaveFName directory
    #
    def plot(self, robotPath, startState, goalState, plotTitle):

        fig = plt.figure()
        ax = fig.add_subplot(111)

        # plotting all the obstacles
        for obstacle in self.obstacles:
            bottomLeftVertex = obstacle[0]
            bottomRightVertex = obstacle[1]
            topRightVertex = obstacle[2]

            width = bottomRightVertex[0] - bottomLeftVertex[0]
            height = topRightVertex[1] - bottomRightVertex[1]
            xy = (bottomLeftVertex[0], bottomLeftVertex[1])

            p = Rectangle(xy, width, height, color='black')
            ax.add_patch(p)

        # plotting the robot's path
        x, y = zip(*robotPath)
        plt.plot(x, y)

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
