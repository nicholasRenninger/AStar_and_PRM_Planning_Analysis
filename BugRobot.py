import yaml
import math
from BugAlgorithms import factory


#
# @brief      Class for the bug robot
#
class BugRobot:

    #
    # @brief      Constructor for the BugRobot object
    #
    # @param      self            The BugRobot object
    # @param      configFileName  The YAML configuration file name which
    #                             contains the start / goal state coords
    # @param      workspace       The workspace object the robot operates in
    # @param      algorithmStr    A string containing name of the control /
    #                             planning algorithmStr the robot uses:
    #                                 - 'bug1'
    #                                 - 'bug2'
    #
    def __init__(self, configFileName, workspace, algorithmStr):

        self.workspace = workspace

        (q0, qG) = self.getStartAndGoalStatesFromFile(configFileName)
        self.startState = q0
        self.goalState = qG

        self.stateHistory = []
        self.currentState = self.startState
        self.stateHistory.append(self.currentState)

        self.rotate(targetState=self.goalState)

        self.timeStepHistory = []

        # defines the linear velocity of the robot when moving
        self.velocityMag = 1

        # define the smallest angle [rad] the robot can rotate by
        NUM_ROT_POSITIONS = 36
        self.turnAngleResolution = (2 * math.pi) / NUM_ROT_POSITIONS

        # 2-norm radius tolerance to be considered "near" an object in
        # workspace
        self.nearObjTolerance = 0.2

        # define how far in front of the robot the bump sensor is
        # this is effectively the distance detection limit, and how far away
        # the robot will stay from obstacles
        self.sensorLengthFromBody = 0.1

        bugAlgorithm = factory.get_BugAlgorithm(self,
                                                algorithmType=algorithmStr)
        self.bugAlgorithm = bugAlgorithm

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

    #
    # @brief      Trys to move BugRobot from startState to goalState using
    #             bugAlgorithm as a controller
    #
    # @param      self  The BugRobot object
    #
    # @return     True if the robot reaches goalState, False if not
    #
    def deploy(self):

        self.bugAlgorithm.controlRobotToGoal()
        return True

    #
    # @brief      moves the robot in the direction of self.currentHeading
    #
    # moves in the current heading at velocityMag for delta_t
    #
    # @param      self     The BugRobot object
    # @param      heading  The heading
    # @param      delta_t  The delta t
    #
    # @return     { description_of_the_return_value }
    #
    def moveForward(self, heading, delta_t):

        velocityMag = self.velocityMag

        deltaX = velocityMag * heading[0] * delta_t
        deltaY = velocityMag * heading[1] * delta_t

        newXPos = self.currentState[0] + deltaX
        newYPos = self.currentState[1] + deltaY
        newRobLoc = [newXPos, newYPos]

        status = self.getCollisionStatus(newRobLoc)

        return (newRobLoc, status)

    #
    # @brief      Determines if at goal.
    #
    # @param      self  The BugRobot object
    #
    # @return     True if at goal, False otherwise.
    #
    def isAtGoal(self):

        closeToGoal = self.isCloseTo(self.goalState)

        if closeToGoal:
            print('reached goal at:', self.goalState)

        return closeToGoal

    #
    # @brief      Determines if robot is close to a given target location
    #
    # @param      self            The BugRobot object
    # @param      targetLocation  The target location
    #
    # @return     True if robot is close to target, False otherwise.
    #
    def isCloseTo(self, targetLocation):
        distToLocation = self.distToTarget(targetLocation)

        return (distToLocation <= self.nearObjTolerance)

    #
    # @brief      computes the distance to the goal state
    #
    # @param      self  The BugRobot object
    #
    # @return     the distance to the goal state of the robot
    #
    def distToGoal(self):
        dist = self.distToTarget(self.goalState)

        return dist

    #
    # @brief      computes the distance to the target
    #
    # @param      self    The BugRobot object
    # @param      target  The target coordinates
    #
    # @return     the distance to the target object
    #
    def distToTarget(self, target):
        (xDist, yDist) = vectorComponentDiff(target, self.currentState)
        distToLocation = vectorMag([xDist, yDist])

        return distToLocation

    #
    # @brief      rotates the robot towards the target state coordinates
    #
    # @param      self         The BugRobot object
    # @param      targetState  The target state coordinates
    #
    # @return     robot points towards targetState, self.currentHeading updated
    #
    def rotate(self, targetState):

        targetHeading = computeHeading(self.currentState, targetState)
        self.currentHeading = targetHeading

    #
    # @brief      rotates the robot to its body frame left by
    #             turnAngleResolution
    #
    # @param      self  The BugRobot object
    #
    # @return     robot rotated to its left by turnAngleResolution,
    #             currentHeading updated
    #
    def rotateLeft(self):

        angle = self.turnAngleResolution
        self.rotateByAngle(angle)
        status = self.getCollisionStatus(self.currentState)

        return status

    #
    # @brief      rotates the robot to its body frame right by
    #             turnAngleResolution
    #
    # @param      self  The BugRobot object
    #
    # @return     robot rotated to its right by turnAngleResolution,
    #             currentHeading updated
    #
    def rotateRight(self):

        angle = -self.turnAngleResolution
        self.rotateByAngle(angle)
        status = self.getCollisionStatus(self.currentState)

        return status

    #
    # @brief      rotates the robot to its left by the given angle [rad]
    #
    # @param      self   The BugRobot object
    # @param      angle  The rotation angle [rad]
    #
    # @return     robot rotated to its left by angle [rad],
    #             currentHeading updated
    #
    def rotateByAngle(self, angle):

        currHeading = self.currentHeading
        currHeadX = currHeading[0]
        currHeadY = currHeading[1]

        c = math.cos(angle)
        s = math.sin(angle)

        newHeadX = currHeadX * c - currHeadY * s
        newHeadY = currHeadY * c + currHeadX * s

        (newHeadX, newHeadY) = normailzeVec(newHeadX, newHeadY)
        self.currentHeading = [newHeadX, newHeadY]

    #
    # @brief      rotates the robot 180 [deg]
    #
    # @param      self   The BugRobot object
    #
    # @return     robot rotated 180[deg],
    #             currentHeading updated
    #
    def turnAround(self):
        self.rotateByAngle(math.pi)
        status = self.getCollisionStatus(self.currentState)

        return status

    #
    # @brief      Gets the collision status.
    #
    # @param      self      The BugRobot object
    # @param      robotLoc  The robot location
    #
    # @return     The collision status.
    #
    def getCollisionStatus(self, robotLoc):

        sensorLoc = self.getSensorLocation(robotLoc)

        # three cases:
        #     - robot and sensor NOT COLLIDED -> all good
        #     - robot and sensor COLLIDED -> need to back up
        #     - robot NOT COLLIDED, sensor COLLIDED -> real collision, stop
        robotCollided = self.workspace.objectCollides(robotLoc)
        sensorCollided = self.workspace.objectCollides(sensorLoc)

        if (not robotCollided) and (not sensorCollided):
            status = 'OK'
        elif robotCollided and sensorCollided:
            status = 'INSIDE_OBSTACLE'
        elif (not robotCollided) and sensorCollided:
            status = 'COLLISION'
        else:
            status = 'UNKNOWN'

        return status

    #
    # @brief      Gets the sensor location.
    #
    # @param      self      The BugRobot object
    # @param      robotLoc  The robot location
    #
    # @return     The sensor location.
    #
    def getSensorLocation(self, robotLoc):
        currHeading = self.currentHeading
        currHeadingX = currHeading[0]
        currHeadingY = currHeading[1]

        currLocX = robotLoc[0]
        currLocY = robotLoc[1]

        sensorX = currLocX + currHeadingX * self.sensorLengthFromBody
        sensorY = currLocY + currHeadingY * self.sensorLengthFromBody

        return [sensorX, sensorY]

    #
    # @brief      Sets the robot state.
    #
    # @param      self           The BugRobot object
    # @param      newRobotState  The new robot state
    #
    # @return     { description_of_the_return_value }
    #
    def setRobotState(self, newRobotState):

        self.currentState = newRobotState
        self.stateHistory.append(newRobotState)


#
# @brief      calulates the component vector difference between vec2 and vec1
#
# @param      vec1  The "start" vector
# @param      vec2  The "end" vector
#
# @return     tuple with (difference in X, difference in Y)
#
def vectorComponentDiff(vec1, vec2):

    diffX = vec2[0] - vec1[0]
    diffY = vec2[1] - vec1[1]

    return (diffX, diffY)


#
# @brief      computes 2-norm vector magnitude of vec
#
# @param      vec   The vector
#
# @return     2-norm vector magnitude of vec
#
def vectorMag(vec):

    x = vec[0]
    y = vec[1]
    magnitude = math.sqrt(x**2 + y**2)

    return magnitude


#
# @brief      Calculates a unit vecor heading from currentPosition to
#             targetObjectPosition
#
# @param      currentPosition       The current position
# @param      targetObjectPosition  The target object position
#
# @return     The heading unit vector between currentPosition and
#             targetObjectPosition
#
def computeHeading(currentPosition, targetObjectPosition):

    (headingX, headingY) = vectorComponentDiff(currentPosition,
                                               targetObjectPosition)

    # need to normalize the heading to get a unit vector heading
    (headingX, headingY) = normailzeVec(headingX, headingY)

    return [headingX, headingY]


#
# @brief      makes a vector with x and y as its components unit length
#
# @note uses 2-norm for normilization
#
# @param      x     x-component of the vector to normalize
# @param      y     y-component of the vector to normalize
#
# @return     magnitude of the vector [x, y]
#
def normailzeVec(x, y):
    mag = vectorMag([x, y])
    newX = x / mag
    newY = y / mag

    return (newX, newY)
