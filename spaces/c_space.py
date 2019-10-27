

# local packages
from spaces.robot_space import RobotSpace


##
# @brief      Concrete implementation of the cspace for a point robot
#
class PointRobotCSpace(RobotSpace):

    ##
    # @brief      PointRobotCSpace class constructor
    #
    # @param      robot            The PointRobot instance
    # @param      workspace        The PointRobot's workspace
    # @param      shouldSavePlots  Boolean controlling whether or not the plt
    #                              objs can be saved to the baseSaveName dir
    # @param      baseSaveFName    The base directory file name for output plot
    #
    # @return     initialized PointRobotCSpace object
    #
    def __init__(self, robot, shouldSavePlots, baseSaveFName):

        # get inherited properties from superclass implementation
        RobotSpace.__init__(self, shouldSavePlots, baseSaveFName)

        self.workspace = robot.workspace
        self.robot = robot

        # as the robot is a point robot, its obstacles are just
        self.obstacles = robot.workspace.obstacles
