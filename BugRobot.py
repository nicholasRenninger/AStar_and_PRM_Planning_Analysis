import BugAlgorithms


class BugRobot:

    def __init__(self, initialState, goalState, algorithm):

        self.stateHistory = []
        self.currentState = initialState
        self.stateHistory.append(self.currentState)
        self.goalState = goalState
        self.algorithm = BugAlgorithms.BugAlgorithm(self,
                                                    algorithmType=algorithm)
