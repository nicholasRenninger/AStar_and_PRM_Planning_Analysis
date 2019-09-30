

#
# @brief      Front-end to create bug algorithm objects with different
#             algorithmTypes
#
class BugAlgorithmFactory:

    #
    # @brief      Return a concrete bug algorithm based on the algorithm type
    #
    # @param      self           The factory object
    # @param      robot          The BugAlgorithm robot
    # @param      algorithmType  The algorithm type string:
    #                                - 'bug1'
    #                                - 'bug2'
    #
    # @return     A concrete, specialized bug algorithm object based on
    #             algorithmType
    #
    def get_BugAlgorithm(self, robot, algorithmType):

        if algorithmType == "bug1":

            return Bug1(robot)

        elif algorithmType == "bug2":

            return Bug2(robot)

        else:
            raise ValueError(algorithmType)


# re-name the module-level class factory for brevity
factory = BugAlgorithmFactory()


#
# @brief      High level class for a "bug" algorithm
#
class BugAlgorithm:

    #
    # @brief      Constructor for the BugAlgorithm object.
    #
    # @param      self           The BugAlgorithm object
    # @param      BugRobot       The BugRobot object instance using this
    #                            algorithm
    #
    def __init__(self, BugRobot):

        self.robot = BugRobot
        self.workspace = BugRobot.workspace

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


#
# @brief      Class for the "bug 1" algorithm, subclassed from BugAlgorithm
#
class Bug1(BugAlgorithm):

    #
    # @brief      Constructs the Bug1 object.
    #
    # @param      self      The Bug1 object
    # @param      BugRobot  The owning BugRobot object
    #
    # @return     The initialized Bug1 algorithm object instance
    #
    def __init__(self, BugRobot):

        BugAlgorithm.__init__(self, BugRobot)


#
# @brief      Class for the "bug 2" algorithm, subclassed from BugAlgorithm
#
class Bug2(BugAlgorithm):

    #
    # @brief      Constructs the Bug1 object.
    #
    # @param      self      The Bug1 object
    # @param      BugRobot  The owning BugRobot object
    #
    # @return     The initialized Bug1 algorithm object instance
    #
    def __init__(self, BugRobot):

        BugAlgorithm.__init__(self, BugRobot)
