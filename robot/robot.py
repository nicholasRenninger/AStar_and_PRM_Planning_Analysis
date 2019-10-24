import math


#
# @brief      This (largely abstract) class describes a generic robot
#
class Robot:

    #
    # @brief      Robot class constructor
    #
    # @param      self             The Robot object object
    # @param      configData       Configuration dictionary for the robot
    # @param      workspace        The Workspace object the robot operates in
    # @param      shouldSavePlots  Boolean controlling whether or not the plt
    #                              objs can be saved to the baseSaveName dir
    # @param      baseSaveFName    The base directory file name for output plot
    #
    # @return     initialized Robot object
    #
    def __init__(self, configData, workspace,
                 shouldSavePlots, baseSaveFName):

        self.workspace = workspace

        # state refers to the robot's actual state vector
        (self.startState,
         self.goalState) = self.initializeRobotState(configData)

        self.stateHistory = []
        self.currentState = self.startState
        self.stateHistory.append(self.currentState)
        self.distTraveled = 0

        self.timeStepHistory = []


    #
    # @brief      returns the start state and goal location lists for the robot
    #
    # @param      configData  configuration data dictionary for the robot
    #
    # @return     The start and goal states lists from configData
    #
    def initializeRobotState(self, configData):

        startState = configData['startState']
        goalState = configData['goalState']

        return (startState, goalState)


    #
    # @brief      abstract method to set the robot shape from the configData
    #
    # @param      configData  configuration data dictionary for the robot
    #
    # @return     robot's shape data in the concrete class will be set 
    #
    def setRobotShape(self, configData):

        raise NotImplementedError


    #
    # @brief      Trys to move BugRobot from startState to goalState using
    #             bugAlgorithm as a controller
    #
    # @param      self  The BugRobot object
    #
    # @return     distance robot traveled
    #
    def deploy(self):

        self.bugAlgorithm.controlRobotToGoal()
        return self.distTraveled

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
    # @brief      Plot all workspace objects and saves to self.baseSaveFName
    #
    #             obstacles, the robot's path, the start location, and the goal
    #             location
    #
    # @param      self        The Workspace object
    #
    # @return     a plot of the manipulator in the self.baseSaveFName directory
    #
    def plot(self, plotTitle):

        raise NotImplementedError


    #
    # @brief      virtual function for a robot instance to "run" itself
    #
    # @param      planner          The Planners object containing the motion
    #                              planning algorithm to be used on the robot
    # @param      shouldSavePlots  Boolean to turn on and off plot file writes
    # @param      baseSaveFName    The base save file name for plots
    #
    # @return     runs robot, then plots results of running the robot
    #
    def runAndPlot(self, planner, shouldSavePlots, baseSaveFName):

        raise NotImplementedError


#
# @brief      calculates the component vector difference between vec2 and vec1
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
# @brief      Calculates a unit vector heading from currentPosition to
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
# @note uses 2-norm for normalization
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