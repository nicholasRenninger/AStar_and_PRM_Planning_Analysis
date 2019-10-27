

##
# @brief      This (largely abstract) class describes a generic motion planner
#
class Planner:

    ##
    # @brief      Planner class constructor
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

        self.plannerType = plannerType
        self.cSpace = cSpace
        self.robot = robot
        self.workspace = workspace

        self.startState = robot.startState
        self.goalState = robot.goalState

    ##
    # @brief      Computes a viable path in robot's cSpace to the goalState
    #             from the robot's startState
    #
    # @return     a viable set of cSpace states from startState to goalState
    #
    def findPathToGoal(self):

        return NotImplementedError
