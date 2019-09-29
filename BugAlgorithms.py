

#
# @brief      High level class for a bug algorithm
#
class BugAlgorithm:

    #
    # @brief      Constructor for the BugAlgorithm object.
    #
    # @param      self           The BugAlgorithm object
    # @param      BugRobot       The BugRobot object instance using this
    #                            algorithm
    # @param      algorithmType  The algorithm type string:
    #                                - 'bug1'
    #                                - 'bug2'
    #
    def __init__(self, BugRobot, algorithmType):

        self.robot = BugRobot
        self.algorithmType = algorithmType

    #
    # @brief      The main bug algorithm state machine controller
    #
    # @param      self  The BugAlgorithm object
    #
    # @return     self.robot will be at the goal
    #
    def controlRobotToGoal(self):

        while not self.robot.isAtGoal:

            bugHitWall = self.moveUntilNewObstacle(self.robot)

            if bugHitWall:

                print('encountered new obstacle at: (',
                      self.robot.currentState, ')')
                self.followObstacle(self.robot)

    #
    # @brief      moves the robot in free workspace until the robot encounters
    #             a new obstacle or reaches the goal
    #
    # @param      self   The BugAlgorithm object
    # @param      robot  An instance of the BugRobot class
    #
    # @return     returns True if the robot hits an obstacle and false if
    #             it reaches the goal
    #
    def moveUntilNewObstacle(self, robot):

        pass

    #
    # @brief      moves the robot along the obstacle until it decides to leave
    #             the obstacle or it reaches the goal
    #
    # @param      self   The BugAlgorithm object
    # @param      robot  An instance of the BugRobot class
    #
    # @return     nothing
    #
    def followObstacle(self, robot):

        pass

    #
    # @brief      A virtual function implementing the specific wall-leaving
    #             behavior for the different self.algorithmType possiblities
    #
    # @param      self   The BugAlgorithm object
    # @param      robot  An instance of the BugRobot class
    #
    # @return     returns True if the robot should leave the obstacle and head
    #             towards the goal, and False if not
    #
    def robotShouldLeaveObstacle(self, robot):

        pass
