# 3rd-party packages
import copy
from shapely.geometry import Point, LineString

# local packages
from robots.robot import Robot
from factory.builder import Builder
from spaces.factory import activeSpaces


##
# @brief      This class describes a point Robot subclass that has no physical
#             dimension in the workspace
#
class PointRobot(Robot):

    ##
    # @brief      PointRobot class constructor
    #
    # @param      robotType        The PointRobot type string
    # @param      configData       Configuration dictionary for the PointRobot
    # @param      workspace        The Workspace object the PointRobot operates
    #                              in
    # @param      shouldSavePlots  Boolean controlling whether or not the plt
    #                              objs can be saved to the baseSaveName dir
    # @param      baseSaveFName    The base directory file name for output plot
    #
    # @return     initialized PointRobot object
    #
    def __init__(self, robotType, configData, workspace,
                 shouldSavePlots, baseSaveFName):

        # calling the superclass contructor to inherit its properties
        Robot.__init__(self,
                       robotType=robotType,
                       configData=configData,
                       workspace=workspace,
                       shouldSavePlots=shouldSavePlots,
                       baseSaveFName=baseSaveFName)

        self.configData = configData

        # need to maintain a separate history for c states and workspace states
        # however, for a point robot, they are the same
        self.cStateHistory = None
        self.startCState = None
        self.goalCState = None
        self.currentState = None
        self.initializeAllRobotStates(configData)

        linearDiscretizationDensity = configData['linearDiscretizationDensity']
        makeSquareCSpace = configData['makeSquareCSpace']

        # have the factory make the PointRobot's C-space
        self.cSpace = activeSpaces.get(robotSpaceType='POINTROBOTCSPACE',
                                       robot=self,
                                       N=linearDiscretizationDensity,
                                       makeSquare=makeSquareCSpace,
                                       shouldSavePlots=shouldSavePlots,
                                       baseSaveFName=baseSaveFName)

    #
    # @brief      Initializes both workspace and cspace robot states.
    #
    # @param      configData  The configuration data
    #
    def initializeAllRobotStates(self, configData):

        self.initializeRobotState(configData)

        self.cStateHistory = []
        self.startCState = self.startState
        self.goalCState = self.goalState

        self.currentState = self.startState
        self.updateRobotState(self.startState)

    ##
    # @brief      Returns whether the given Shapely object collides with any
    #             workspace obstacles
    #
    # @param      shapelyObject  The shapely object to check for collision
    # @param      gridIndices    The grid indices of the shapely object (not
    #                            used here)
    #
    # @return     True if the shapely object collides with any obstacle, False
    #             otherwise
    #
    def checkCollision(self, shapelyObject, gridIndices=None):

        obstacles = self.workspace.polygonObstacles

        for obstacle in obstacles:

            if obstacle.intersects(shapelyObject):

                return True

        return False

    ##
    # @brief      Returns whether the CSpace state collides with any workspace
    #             obstacles
    #
    # @param      state  The configuration space state as a numpy list
    #
    # @return     True if the CSpace state collides with any obstacle, False
    #             otherwise
    #
    def checkCollisionWithState(self, state):

        pointToCheck = Point(state)
        collided = self.checkCollision(pointToCheck)

        return collided

    ##
    # @brief      Checks whether the line connecting the two states collides
    #             with anything
    #
    # @param      startState  The start state in Cspace as an np array
    # @param      endState    The end state in Cspace as an np array
    #
    # @return     boolean determining whether following a line between the
    #             start and end states will cause you to collide with an
    #             obstacle
    #
    def checkLinearPathCollision(self, startState, endState):

        start = tuple(startState.flatten())
        end = tuple(endState.flatten())
        lineToCheck = LineString([start, end])

        collided = self.checkCollision(lineToCheck)
        return collided

    ##
    # @brief      Puts the robot into the newState and updates appropriate data
    #
    # @param      newState  The new state for the PointRobot
    #
    def updateRobotState(self, newState):

        self.stateHistory.append(copy.deepcopy(newState))
        self.cStateHistory.append(copy.deepcopy(newState))
        self.distTraveled += self.distToTarget(self.currentState, newState)
        self.currentState = newState

    ##
    # @brief      Plots the robot's body's path in the workspace
    #
    # @note       this function does nothing, as the PointRobot has no body
    #
    # @param      ax    the matplotlib.axes object to plot the PointRobot's
    #                   body's path in its Workspace
    # @param      fig   The matplotlib fig object to plot on
    #
    # @return     plots the PointRobot's body's path on ax
    #
    def plotBodyInWorkspace(self, ax, fig):

        return None

    ##
    # @brief      Function for PointRobot instance to "run" itself
    #
    #             Runs the planning algorithm, reports if it found a path,
    #             collects benchmarking info, and plots the solution if desired
    #
    # @param      planner            The planner object containing the motion
    #                                planning algorithm to be used on the robot
    # @param      plotTitle          The plot title string
    # @param      plotPlannerOutput  A flag to tell the run whether to run its
    #                                plotting routines
    # @param      shouldBenchmark    Flag to turn on / off benchmarking
    #                                reporting
    # @param      plannerConfigData  The planner configuration data
    #
    # @return     (a dictionary with the benchmarking performance and planner
    #             settings used in this run, a boolean flag indicating whether
    #             or not a path was found)
    #
    def runAndPlot(self, planner, plotTitle, plotPlannerOutput=True,
                   shouldBenchmark=False, plannerConfigData=None):

        # need to reset everything about the robot's state when it's run
        self.initializeAllRobotStates(self.configData)

        plotConfigData = {'shouldPlot': plotPlannerOutput,
                          'plotTitle': plotTitle,
                          'xlabel': 'x',
                          'ylabel': 'y',
                          'plotObstacles': True,
                          'plotGrid': False}

        # don't print statuses when benchmarking as it floods the terminal
        if not shouldBenchmark:
            print('Starting ', planner.plannerType, ' planner ...')

        (computationTime,
         pathLength,
         fp) = planner.findPathToGoal(startState=self.startCState,
                                      goalState=self.goalCState,
                                      plotConfigData=plotConfigData,
                                      plannerConfigData=plannerConfigData,
                                      shouldBenchmark=shouldBenchmark)
        foundPath = fp

        # don't print statuses when benchmarking as it floods the terminal
        if not shouldBenchmark:

            if foundPath:
                print('Reached goal at:', self.stateHistory[-1])
                print('Path length: ', self.distTraveled)
            else:
                print('No valid path found with current planner:',
                      planner.plannerType)

        if shouldBenchmark:

            # return this benchmarking info as a dictionary so it can be loaded
            # as a pandas dataframe later
            data = {'computationTimeInSeconds': computationTime,
                    'pathLength': pathLength}
            bencmarkingInfo = {**data, **plannerConfigData}

        else:

            bencmarkingInfo = None

        # allow the robot to run without plotting all of its spatial
        # representations
        if plotPlannerOutput:

            plotConfigData = {'plotTitle': plotTitle + 'workspace',
                              'xlabel': 'x',
                              'ylabel': 'y'}
            self.workspace.plot(robot=self,
                                startState=self.startState,
                                goalState=self.goalState,
                                plotConfigData=plotConfigData)

            plotConfigData = {'plotTitle': plotTitle + 'cSpace',
                              'xlabel': 'x',
                              'ylabel': 'y',
                              'plotObstacles': True,
                              'plotGrid': True}
            self.cSpace.plot(robot=self,
                             startState=self.startState,
                             goalState=self.goalState,
                             plotConfigData=plotConfigData)

        return (bencmarkingInfo, foundPath)


##
# @brief      Implements the generic builder class for the PointRobot
#
class PointRobotBuilder(Builder):

    ##
    # @brief      need to call the super class constructor to gain its
    #             properties
    #
    def __init__(self):
        Builder.__init__(self)

    ##
    # @brief      Implements the smart constructor for PointRobot
    #
    #             Only reads the config data once, otherwise just returns the
    #             built object
    #
    # @param      robotType        The PointRobot type string
    # @param      configFileName   The configuration file name
    # @param      workspace        The Workspace object the PointRobot operates
    #                              in
    # @param      shouldSavePlots  Boolean controlling whether or not the plt
    #                              objs can be saved to the baseSaveName dir
    # @param      baseSaveFName    The base directory file name for output plot
    #
    # @return     instance of an initialized PointRobot object
    #
    def __call__(self, robotType, configFileName, workspace, shouldSavePlots,
                 baseSaveFName):

        configNameHasChanged = (self._configName != configFileName)
        noInstanceLoadedYet = (self._instance is None)

        if noInstanceLoadedYet or configNameHasChanged:

            configData = self.loadConfigData(configFileName)
            self._instance = PointRobot(robotType=robotType,
                                        configData=configData,
                                        workspace=workspace,
                                        shouldSavePlots=shouldSavePlots,
                                        baseSaveFName=baseSaveFName)

        return self._instance
