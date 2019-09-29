import yaml
import BugAlgorithms


#
# @brief      Class for the bug robot
#
class BugRobot:

    #
    # @brief      Constructor for the BugRobot object
    #
    # @param      self            The BugRobot object
    # @param      configFileName  The YAML configuration file name
    #                             which contains the start / goal state coords
    # @param      workspace       The workspace object the robot operates in
    # @param      algorithm       A string containing name of the control
    #                             / planning algorithm the robot uses:
    #                                 - 'bug1'
    #                                 - 'bug2'
    #
    def __init__(self, configFileName, workspace, algorithm):

        self.workspace = workspace

        (q0, qG) = self.getStartAndGoalStatesFromFile(configFileName)
        self.startState = q0
        self.goalState = qG

        self.stateHistory = []
        self.currentState = self.startState
        self.stateHistory.append(self.currentState)

        self.algorithm = BugAlgorithms.BugAlgorithm(self,
                                                    algorithmType=algorithm)

    #
    # @brief      returns the start and goal location lists for the robot
    #
    # @param      configFileName  The YAML configuration file name
    #
    # @return     The start and goal states lists from the config filename
    #
    @staticmethod
    def getStartAndGoalStatesFromFile(configFileName):

        with open(configFileName, 'r') as stream:
            config_data = yaml.safe_load(stream)

        startLoc = config_data['qStart']
        goalLoc = config_data['qGoal']

        return (startLoc, goalLoc)
