# 3rd-party packages
import os.path
import itertools

# local packages
from spaces.factory import activeSpaces
from robots.factory import activeRobots
from planners.factory import availablePlanners
from factory.builder import Builder


##
# @brief      This class allows for the creation and running of a motion
#             planning simulation based on the simulation type string
#
class Simulation:

    def __init__(self, shouldSavePlots, basePlotDir):

        self.shouldSavePlots = shouldSavePlots

        self.basePlotDir = basePlotDir

    ##
    # @brief      Runs a set of simulations based on the simulation type and
    #             outputs the plots to the
    #
    # @param      simType  The simulation type string
    #
    def run(self, simType):

        (configNames, configFileNames) = self.getConfigPaths(simType)

        for (file, configName) in zip(configFileNames, configNames):

            print('===============================')
            print(configName)
            print('===============================')

            baseSaveFName = os.path.join(self.basePlotDir, configName)

            if simType == 'polygonalRobot':

                self.runRobotWithPlanner(robotType='POLYGONALROBOT',
                                         plannerType=None,
                                         runPlannerBenchmarking=False,
                                         configFileName=file,
                                         baseSaveFName=baseSaveFName)

            if simType == 'gradient':

                self.runRobotWithPlanner(robotType='POINTROBOT',
                                         plannerType='GRADIENT',
                                         runPlannerBenchmarking=False,
                                         configFileName=file,
                                         baseSaveFName=baseSaveFName)

            if simType == 'wavefront':

                self.runRobotWithPlanner(robotType='POINTROBOT',
                                         plannerType='WAVEFRONT',
                                         runPlannerBenchmarking=False,
                                         configFileName=file,
                                         baseSaveFName=baseSaveFName)

            if simType == 'manipulator':

                self.runRobotWithPlanner(robotType='MANIPULATOR',
                                         plannerType='WAVEFRONT',
                                         runPlannerBenchmarking=False,
                                         configFileName=file,
                                         baseSaveFName=baseSaveFName)

            if simType == 'graphSearch':

                self.runGraphSearch(file, baseSaveFName)

            if (simType == 'prmPointRobot' or
               simType == 'prmPointRobotBenchmark'):

                runBenchMarks = (simType == 'prmPointRobotBenchmark')

                self.runRobotWithPlanner(robotType='POINTROBOT',
                                         plannerType='PRM',
                                         runPlannerBenchmarking=runBenchMarks,
                                         configFileName=file,
                                         baseSaveFName=baseSaveFName)

    ##
    # @brief      Generic function to interface with the classes to run a
    #             motion planner on a robot in variety of environments
    #
    # @param      robotType               The robot type string
    # @param      plannerType             The planner type string
    # @param      runPlannerBenchmarking  Boolean flag indicating whether the
    #                                     planner should be benchmarked
    #                                     according to the settings in the
    #                                     simulation configuration files
    # @param      configFileName          The configuration file name for the
    #                                     simulation
    # @param      baseSaveFName           The base save file name for plots
    #
    def runRobotWithPlanner(self, robotType, plannerType,
                            runPlannerBenchmarking, configFileName,
                            baseSaveFName):

        ssp = self.shouldSavePlots

        # the workspace doesn't change for this simulation
        currWorkspace = activeSpaces.get(robotSpaceType='WORKSPACE',
                                         configFileName=configFileName,
                                         shouldSavePlots=ssp,
                                         baseSaveFName=baseSaveFName)

        currRobot = activeRobots.get(robotType=robotType,
                                     configFileName=configFileName,
                                     workspace=currWorkspace,
                                     shouldSavePlots=ssp,
                                     baseSaveFName=baseSaveFName)

        # gaurds against trying to plan for a robot that does not support any
        # planners yet
        currPlanner = None
        if plannerType:

            currPlanner = availablePlanners.get(plannerType=plannerType,
                                                cSpace=currRobot.cSpace,
                                                workspace=currWorkspace,
                                                robot=currRobot,
                                                configFileName=configFileName,
                                                shouldSavePlots=ssp,
                                                baseSaveFName=baseSaveFName)

        # if benchmarking, use the same workspace, cspace, and planner, just
        # adjust settings in planner for each experiment
        if plannerType and runPlannerBenchmarking:

            self.runPlannerBenchmarking(planner=currPlanner,
                                        cSpace=currRobot.cSpace,
                                        workspace=currWorkspace,
                                        robot=currRobot,
                                        configFileName=configFileName,
                                        baseSaveFName=baseSaveFName)

        # execute robot with whatever planner is given, even if planner is
        # still None
        if not runPlannerBenchmarking:

            currRobot.runAndPlot(planner=currPlanner, plotTitle='')

    ##
    # @brief      Generic function to run a robot in the same workspace many
    #             times by varying the planner / planner parameters and
    #             reporting statistical analysis of the runs
    #
    # @param      planner         The initialized planner object for the
    #                             simulation scenario
    # @param      cSpace          The configuration space of the robot
    # @param      workspace       The workspace object the robot operates in
    # @param      robot           The Robot object to use
    # @param      configFileName  The configuration file name
    # @param      baseSaveFName   The base directory file name for output plot
    #
    def runPlannerBenchmarking(self, planner, cSpace, workspace, robot,
                               configFileName, baseSaveFName):

        shouldSavePlots = self.shouldSavePlots

        # need to load the config data here to extract simulation benchmarking
        # parameters
        configData = Builder.loadConfigData(configFileName)
        numRunsOfPlannerPerSetting = configData['numRunsOfPlannerPerSetting']

        # this sets which of the planner setting will be changed. We need to
        # create all possible combinations of settings to use for all of the
        # planner runs
        parametersToVary = configData['paramterNamesToVary']

        # builds a dict of variable names and the associated lists of
        # parameters to use from the config file
        allParams = dict((var, configData[var]) for var in parametersToVary)

        # building a list of all possible combinations of values for each key
        # in the dictionary of lists of parameter lists
        keys, values = zip(*allParams.items())
        experiments = [dict(zip(keys, v)) for v in itertools.product(*values)]

        # now that we have a unique dictionary of planner settings for desired
        # experiment, we now need to run each of these settings combinations
        # numRunsOfPlannerPerSetting times, then average the statistics across
        # all runs
        for i in range(0, numRunsOfPlannerPerSetting):

            for experiment in experiments:

                pass

    ##
    # @brief      A function to interface with the graph class and demonstrate
    #             the performance of both the A* and Dijkstra optimal search
    #             algorithms
    #
    # @param      configFileName  The configuration file name for the
    #                             simulation
    # @param      baseSaveFName   The base save file name for plots
    #
    def runGraphSearch(self, configFileName, baseSaveFName):

        pass

    ##
    # @brief      Gets the configuration file paths for the  given sim type
    #
    # @param      simType  The simulation type string
    #
    # @return     The configuration path strings in a list
    #
    def getConfigPaths(self, simType):

        if simType == 'polygonalRobot':
            configNames = ['WO_Rob_triangles', 'WO_Rob_triangles_no_rot']

        elif simType == 'gradient':
            configNames = ['env1', 'env2', 'env3']

        elif simType == 'wavefront':
            configNames = ['env2', 'env3']

        elif simType == 'manipulator':
            configNames = ['env1', 'env2', 'env3']

        elif simType == 'graphSearch':
            configNames = ['']

        elif simType == 'prmPointRobot':
            configNames = ['env1']

        elif simType == 'prmPointRobotBenchmark':
            configNames = ['env1']

        else:
            raise ValueError(simType)

        fullConfigNames = list(map(lambda x: simType + '_' + x, configNames))

        configDir = 'config'
        fType = '.yaml'
        configFileNames = [os.path.join(configDir, fName) + fType
                           for fName in fullConfigNames]

        return (fullConfigNames, configFileNames)
