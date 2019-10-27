

##
# @brief      This (largely abstract) class describes a generic motion planner
#
class Planner:

    ##
    # @brief      Planner class constructor
    #
    # @param      plannerType  The planner type
    # @param      cSpace       The configuration space of the robot space
    # @param      workspace    The workspace object the robot operates in
    # @param      robot        The Planner type @string
    # @param      configData   Configuration dictionary for the planner
    #
    # @return     initialized Planner object
    #
    def __init__(self, plannerType, cSpace, workspace, robot, configData):

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

    def bitchImACow(self, meow):
    

