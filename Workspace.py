import yaml
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle


# @brief      Class for workspace objects for a 2D robot
#
#  Workspace can only be comprised of rectangular obstacles, a
#  start location, and a goal location
class Workspace:

    #
    # @brief      Workspace class constructor
    #
    # @param      self            The workspace object object
    # @param      configFileName  The workspace configuration file name
    #                             contains the obstacle coords, and the start /
    #                             goal coords
    #
    # @return     initialized workspace object
    #
    def __init__(self, configFileName, shouldSavePlots, baseSaveFName):

        (startLoc, goalLoc, obstacles) = self.getEnvFromFile(configFileName)
        self.obstacles = obstacles
        self.startLoc = startLoc
        self.goalLoc = goalLoc

        self.shouldSavePlots = shouldSavePlots
        self.baseSaveFName = baseSaveFName

    #
    # @brief      Creates a list of obstacles from a YAML config file
    #
    # @return     (start location coords, goal location coords,
    #              a list of workspace obstacle vertex coordinates for each
    #              obstacle)
    #
    @staticmethod
    def getEnvFromFile(configFileName):

        with open(configFileName, 'r') as stream:
            config_data = yaml.safe_load(stream)

        startLoc = config_data['qStart']
        goalLoc = config_data['qGoal']
        obstacles = config_data['WO']
        return (startLoc, goalLoc, obstacles)

    #
    # @brief      Plots of all workspace objects and saves to ../figures
    #
    # obstacles, the robot's path, the start location, and the goal location
    #
    # @param      self  The workspace object
    #
    # @return     a plot of the workspace in the ../figures directory
    #
    def plot(self, robotPath, plotTitle):

        fig = plt.figure()
        ax = fig.add_subplot(111)

        for obstacle in self.obstacles:
            bottomLeftVertex = obstacle[0]
            bottomRightVertex = obstacle[1]
            topRightVertex = obstacle[2]

            width = bottomRightVertex[0] - bottomLeftVertex[0]
            height = topRightVertex[1] - bottomRightVertex[1]
            xy = (bottomLeftVertex[0], bottomLeftVertex[1])

            p = Rectangle(xy, width, height, color='black')
            ax.add_patch(p)

        x, y = zip(*robotPath)
        plt.plot(x, y)

        ax.set_axis_off()

        if self.shouldSavePlots:
            saveFName = self.baseSaveFName + '-' + plotTitle + '.png'
            fig = plt.gcf()
            fig.canvas.manager.full_screen_toggle()
            fig.show()
            fig.set_size_inches((11, 8.5), forward=False)
            plt.savefig(saveFName, dpi=500)
            print('wrote figure to ', saveFName)
