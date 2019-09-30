

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

        if algorithmType == 'bug1':

            return Bug1(robot)

        elif algorithmType == 'bug2':

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

        while not self.robot.isAtGoal():

            bugHitWall = self.moveUntilNewObstacle()
            return True

            # if bugHitWall:

            #     print('encountered new obstacle at: (',
            #           self.robot.currentState, ')')
            #     self.followObstacle(self.robot)

    #
    # @brief      moves the robot in free workspace until the robot encounters
    #             a new obstacle or reaches the goal
    #
    # @param      self  The BugAlgorithm object
    #
    # @return     returns True if the robot hits an obstacle and false if it
    #             reaches the goal
    #
    def moveUntilNewObstacle(self):

        self.robot.rotateToGoal()

        delta_t = 0.01

        # define multiplier on delta_t when robot motion simulation causes the
        # non-physical behavior of the robot colliding with an object
        scalingFactor = 0.9

        currHeading = self.robot.currentHeading
        currRobLoc = self.robot.currentState

        status = self.robot.getCollisionStatus(currRobLoc)

        while status != 'COLLISION':

            if self.robot.isAtGoal():
                return False

            # now need to keep going forward until and actual collision
            # happens, where the robot tactile sensor collides with the wall,
            # but not the robot. To accomplish this, adjust the delta_t until
            # the robot just collides with the obstacle

            (newX, newY) = self.robot.moveForward(currHeading,
                                                  delta_t)
            newRobLoc = [newX, newY]

            status = self.robot.getCollisionStatus(newRobLoc)

            if status == 'INSIDE_OBSTACLE':
                delta_t *= scalingFactor

            elif status == 'OK':
                currRobLoc = newRobLoc
                self.robot.setRobotState(newRobLoc)

            elif status == 'UNKNOWN':
                raise ValueError(status)

        return True

    #
    # @brief      moves the robot along the obstacle until it decides to leave
    #             the obstacle or it reaches the goal
    #
    # @param      self  The BugAlgorithm object
    #
    # @return     nothing
    #
    def followObstacle(self):

        pass

    #
    # @brief      A virtual function implementing the specific wall-leaving
    #             behavior for the different self.algorithmType possiblities
    #
    # @param      self  The BugAlgorithm object
    #
    # @return     returns True if the robot should leave the obstacle and head
    #             towards the goal, and False if not
    #
    def robotShouldLeaveObstacle(self):

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
