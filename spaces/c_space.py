# local packages
from spaces.robot_space import RobotSpace


#
# @brief      Class implementing a PolygonalRobot's configuration space object
#
class PolygonalRobotCSpace(RobotSpace):

    #
    # @brief      PolygonalRobotCSpace class constructor
    #
    # @param      robot            The PolygonalRobot instance
    # @param      workspace        The PolygonalRobot's workspace
    # @param      shouldSavePlots  Boolean controlling whether or not the plt
    #                              objs can be saved to the baseSaveName dir
    # @param      baseSaveFName    The base directory file name for output plot
    #
    # @return     initialized PolygonalRobotCSpace object
    #
    def __init__(self, robot, shouldSavePlots, baseSaveFName):

        # get inherited properties from superclass implementation
        RobotSpace.__init__(self, shouldSavePlots, baseSaveFName)
        
        self.workspace = robot.workspace
        self.robot = robot

        obstacles = self.getObstacles(robot, self.workspace)
        self.obstacles = obstacles


    #
    # @brief      Calculates PolygonalRobotCSpace obstacles from the robot type and the
    #             obstacles in the workspace
    #
    # @param      robot      The robot
    # @param      workspace  The workspace
    # @param      configData configuration data dictionary for the robot
    #
    # @return     a list of PolygonalRobotCSpace obstacle vertex coordinates for each
    #             obstacle
    #
    def getObstacles(self, robot, workspace):

        return None


