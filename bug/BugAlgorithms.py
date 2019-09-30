from . import BugRobot


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
    # @param      self      The BugAlgorithm object
    # @param      BugRobot  The BugRobot object instance using this algorithm
    #
    def __init__(self, BugRobot):

        self.robot = BugRobot
        self.workspace = BugRobot.workspace

        # define multiplier on delta_t when robot motion simulation causes the
        # non-physical behavior of the robot colliding with an object
        self.delta_t = 0.1
        self.scalingFactor = 0.8

    #
    # @brief      The main bug algorithm state machine controller
    #
    # @param      self  The BugAlgorithm object
    #
    # @return     self.robot will be at the goal
    #
    def controlRobotToGoal(self):

        while not self.robot.isAtGoal():

            print('------------')
            bugHitWall = self.moveUntilNewObstacle()

            if bugHitWall:
                self.followObstacle()
                print('------------')

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

        print('moving towards goal')
        self.robot.rotate(targetState=self.robot.goalState)

        delta_t = self.delta_t
        scalingFactor = self.scalingFactor

        currentHeading = self.robot.currentHeading
        currRobLoc = self.robot.currentState

        status = self.robot.getCollisionStatus(currRobLoc)

        while status != 'COLLISION':

            if self.robot.isAtGoal():
                return False

            # now need to keep going forward until and actual collision
            # happens, where the robot tactile sensor collides with the wall,
            # but not the robot. To accomplish this, decrease delta_t until
            # the robot just collides with the obstacle
            (newRobLoc, status) = self.robot.moveForward(currentHeading,
                                                         delta_t)

            if status == 'INSIDE_OBSTACLE':
                delta_t *= scalingFactor

            elif status == 'OK':
                currRobLoc = newRobLoc
                self.robot.setRobotState(newRobLoc)
                delta_t = self.delta_t

            # need to save the state when it actually collides
            elif status == 'COLLISION':
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

        print('following obstacle')
        delta_t = self.delta_t
        (newRobLoc, status) = self.robot.moveForward(self.robot.currentHeading,
                                                     delta_t)
        while not self.shouldLeaveObstacle():
            # hit the obstacle, rotate 180[deg], then turn left until the force
            # sensor collides with the wall, then rotate back right, then move
            # forwards again until you hit the wall again, repeat
            status = self.robot.turnAround()

            # robot tactile sensor has hit the wall
            while status != 'COLLISION':
                status = self.robot.rotateRight()

            # rotate back from the wall until its clear to go forward again
            while status != 'OK':
                status = self.robot.rotateLeft()

            # go forward with the current heading as long as you can until the
            # tactile sensor once again bumps the wall
            currentHeading = self.robot.currentHeading
            (newRobLoc, status) = self.robot.moveForward(currentHeading,
                                                         delta_t)
            self.robot.setRobotState(newRobLoc)

    #
    # @brief      A virtual function implementing the specific wall-leaving
    #             behavior for the different self.algorithmType possibilities
    #
    # @param      self  The BugAlgorithm object
    #
    # @return     returns True if the robot should leave the obstacle and head
    #             towards the goal, and False if not
    #
    def shouldLeaveObstacle(self):

        raise NotImplementedError


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
        self.resetAlgorithm()

    #
    # @brief      computes whether or not the robot should leave the obstacle
    #
    # @note this function is what makes "bug1" unique from "bugXX"
    #
    # used during obstacle following to determine where the best place would be
    # to start moving towards the goal again
    #
    # @param      self  The Bug1 object
    #
    # @return     True if the robot should leave the obstacle and continue to
    #             the goal, False, if it should continue exploring the obstacle
    #
    def shouldLeaveObstacle(self):

        currLoc = self.robot.currentState
        distToGoal = self.robot.distToGoal()

        # robot just reached obstacle
        if not all(self.coordsNearestGoal):
            print('encountered new obstacle at:', currLoc)
            self.coordsFirstEncounteredObstacle = currLoc
            self.coordsNearestGoal = currLoc

        # updating closest point on the obstacle to the goal
        if distToGoal < self.shortestDistanceToGoalOnObst:
            print('New coords on obstacle closest to goal: ', currLoc)
            self.shortestDistanceToGoalOnObst = distToGoal
            self.coordsNearestGoal = currLoc

        startLoc = self.coordsFirstEncounteredObstacle
        robotCloseToFirstLocOnObstacle = self.robot.isCloseTo(startLoc)

        # robot has begun exploring obstacle
        if not self.exploringObstacle and not robotCloseToFirstLocOnObstacle:
            print('Robot now exploring obstacle')
            self.exploringObstacle = True

        # robot has explored the whole obstacle
        elif self.exploringObstacle and robotCloseToFirstLocOnObstacle:
            print('Robot has explored whole obstacle')
            self.exploredWholeObstacle = True

        # robot has explored whole object and has returned to the starting
        # location, should signal state machine to transition to next state
        closeLoc = self.coordsNearestGoal
        if self.exploredWholeObstacle and self.robot.isCloseTo(closeLoc):
            print('robot leaving obstacle from: ', currLoc)
            self.resetAlgorithm()
            return True
        else:
            return False

    #
    # @brief      resets all internal parameters unique to Bug1 to default
    #
    # @param      self  The Bug1 object
    #
    # @return     all unique Bug1 attributes are reset to default
    #
    def resetAlgorithm(self):
        self.coordsFirstEncounteredObstacle = [None, None]
        self.coordsNearestGoal = [None, None]
        self.exploredWholeObstacle = False
        self.exploringObstacle = False
        self.shortestDistanceToGoalOnObst = float('inf')


#
# @brief      Class for the "bug 2" algorithm, subclassed from BugAlgorithm
#
class Bug2(BugAlgorithm):

    #
    # @brief      Constructs the Bug2 object.
    #
    # @param      self      The Bug2 object
    # @param      BugRobot  The owning BugRobot object
    #
    # @return     The initialized Bug2 algorithm object instance
    #
    def __init__(self, BugRobot):

        BugAlgorithm.__init__(self, BugRobot)
        self.resetAlgorithm()

    #
    # @brief      computes whether or not the robot should leave the obstacle
    #
    # @note this function is what makes "bug2" unique from "bugXX"
    #
    # used during obstacle following to determine where the best place would be
    # to start moving towards the goal again
    #
    # @param      self  The Bug2 object
    #
    # @return     True if the robot should leave the obstacle and continue to
    #             the goal, False, if it should continue exploring the obstacle
    #
    def shouldLeaveObstacle(self):

        currLoc = self.robot.currentState

        # robot just reached obstacle
        if not all(self.coordsFirstEncounteredObstacle):
            print('encountered new obstacle at:', currLoc)
            self.coordsFirstEncounteredObstacle = currLoc
            self.distToGoalFromStart = self.robot.distToGoal()
            return False

        firstEncountered = self.coordsFirstEncounteredObstacle
        currDist = self.robot.distToGoal()
        currLocCloserToGoalThanStart = currDist < self.distToGoalFromStart

        notAtStart = not self.robot.isCloseTo(firstEncountered)

        if self.robotOnMLine() and currLocCloserToGoalThanStart and notAtStart:
            print('robot leaving obstacle from: ', currLoc)
            self.resetAlgorithm()
            return True

        return False

    #
    # @brief      determines if robot is currently on the m-line
    #
    # m-line is the straight line from the start to goal state
    #
    # @param      self  The Bug2 object
    #
    # @return     True if the robots current position is close to the position
    #             it would be at if it were on the m-line, False otherwise
    #
    def robotOnMLine(self):

        startLoc = self.robot.startState
        goalLoc = self.robot.goalState
        startLocY = startLoc[1]

        headingToGoal = BugRobot.computeHeading(startLoc, goalLoc)
        mLineSlope = headingToGoal[1] / headingToGoal[0]

        robotX = self.robot.currentState[0]

        mLineY = startLocY + robotX * mLineSlope
        mLineX = robotX

        mLinePosAtRobotX = [mLineX, mLineY]

        return self.robot.isCloseTo(mLinePosAtRobotX)

    #
    # @brief      resets all internal parameters unique to Bug2 to default
    #
    # @param      self  The Bug2 object
    #
    # @return     all unique Bug1 attributes are reset to default
    #
    def resetAlgorithm(self):
        self.coordsFirstEncounteredObstacle = [None, None]
        self.distToGoalFromStart = float('inf')
