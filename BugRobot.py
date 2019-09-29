import yaml
import BugAlgorithms


class BugRobot:

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

    @staticmethod
    def getStartAndGoalStatesFromFile(configFileName):

        with open(configFileName, 'r') as stream:
            config_data = yaml.safe_load(stream)

        startLoc = config_data['qStart']
        goalLoc = config_data['qGoal']

        return (startLoc, goalLoc)
