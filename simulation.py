# 3rd-party packages
import os.path
import itertools
import pandas as pd
import copy
import matplotlib.pyplot as plt
import seaborn as sns

# local packages
from spaces.factory import activeSpaces
from robots.factory import activeRobots
from planners.factory import availablePlanners
from factory.builder import Builder
from util.plots import savePlot
from spaces.graph import Graph


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

                doBench = (simType == 'prmPointRobotBenchmark')

                self.runRobotWithPlanner(robotType='POINTROBOT',
                                         plannerType='PRM',
                                         runPlannerBenchmarking=doBench,
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
        confName = configFileName

        # the workspace doesn't change for this simulation
        currWorkspace = activeSpaces.get(robotSpaceType='WORKSPACE',
                                         configFileName=confName,
                                         shouldSavePlots=ssp,
                                         baseSaveFName=baseSaveFName)

        currRobot = activeRobots.get(robotType=robotType,
                                     configFileName=confName,
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
                                                configFileName=confName,
                                                shouldSavePlots=ssp,
                                                baseSaveFName=baseSaveFName)

            # if benchmarking, use the same workspace, cspace, and planner,
            # just adjust settings in planner for each experiment
            if runPlannerBenchmarking:

                data = self.runPlannerBenchmarking(planner=currPlanner,
                                                   robot=currRobot,
                                                   configFileName=confName)

                (benchMarkingDF, pathValidityDF, benchParams) = data
                plotTitle = plannerType + '_stats'
                self.plotStatistics(benchMarkingDF=benchMarkingDF,
                                    pathValidityDF=pathValidityDF,
                                    benchParams=benchParams,
                                    baseSaveFName=baseSaveFName,
                                    plotTitle=plotTitle)

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
    # @param      robot           The Robot object to use
    # @param      configFileName  The configuration file name
    #
    # @return     (pandas data frame with the computation time and path length
    #             of each run for each paramteric experimental setting
    #             specified in the configuration file (e.g.):
    #                   computationTime pathLength    n    r
    #                    1.702000e-06   23.814193     200  0.5
    #                    6.310000e-07   21.638431     200  0.5
    #
    #             pandas data frame with the number of valid paths per
    #             experimental paramter set and the number of times the planner
    #             tried to find a path:
    #                   numValidPaths    n    r
    #                               0  200  0.5
    #                               0  200  1.0
    #
    #             list of strings of the parameters varied in benchmarking)
    #
    def runPlannerBenchmarking(self, planner, robot, configFileName):

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
        data = []
        pathValidityData = []
        for experiment in experiments:

            print('benchmarking the ', planner.plannerType, ' planner ',
                  numRunsOfPlannerPerSetting, ' times with:', experiment)

            # we would like to count how many times each experiment produces a
            # valid path to the goal in cspace
            numValidPaths = 1
            runInfo = {}

            for i in range(0, numRunsOfPlannerPerSetting):

                (benchmarkingInfo,
                 foundPath) = robot.runAndPlot(planner=planner,
                                               plotPlannerOutput=False,
                                               plotTitle='',
                                               shouldBenchmark=True,
                                               plannerConfigData=experiment)

                # add the current experiment parameters to the dictionary for
                # creating the data frame later
                benchmarkingInfo.update(experiment)
                data.append(benchmarkingInfo)

                if foundPath:
                    numValidPaths += 1

            # add how many times a valid path was found to the benchmarking
            # info so we can later analyze the efficacy of our experimental
            # settings
            runInfo['numValidPaths'] = copy.deepcopy(numValidPaths)
            runInfo['numTimesRun'] = numRunsOfPlannerPerSetting
            runInfo.update(copy.deepcopy(experiment))
            pathValidityData.append(runInfo)

        # easier to do stat analysis with a dataframe
        benchMarkingDF = pd.DataFrame(data)
        pathValidityDF = pd.DataFrame(pathValidityData)

        return (benchMarkingDF, pathValidityDF, parametersToVary)

    ##
    # @brief      Plots the statistics for the dataframes from the benchmark
    #             and path validity (how often the probabilistic planner is
    #             able to find a path)
    #
    # @param      benchMarkingDF  pandas data frame with the computation time
    #                             and path length of each run for each
    #                             paramteric experimental setting specified in
    #                             the configuration file (e.g.):
    #                             computationTime pathLength    n    r
    #                             1.702000e-06   23.814193     200  0.5
    #                             6.310000e-07   21.638431     200  0.5
    #
    # @param      pathValidityDF  pandas data frame with the number of valid
    #                             paths per experimental paramter set and the
    #                             number of times the planner tried to find a
    #                             path
    #                             numValidPaths    n    r
    #                                         0  200  0.5
    #                                         0  200  1.0
    #
    # @param      benchParams     list of strings of the parameters varied in
    #                             benchmarking
    # @param      baseSaveFName   The base directory file name for output plot
    # @param      plotTitle       The plot title
    #
    def plotStatistics(self, benchMarkingDF, pathValidityDF, benchParams,
                       baseSaveFName, plotTitle):

        ##
        # Plotting boxplots
        ##
        boxPlotsToMake = ['computationTime', 'pathLength']

        # need to create a new, merged categorical data for boxplots
        mergedParamsName = ', '.join(benchParams)
        benchMarkingDF[mergedParamsName] = benchMarkingDF[benchParams].apply(
            lambda x: ', '.join(x.astype(str)), axis=1)
        pathValidityDF[mergedParamsName] = pathValidityDF[
            benchParams].apply(lambda x: ', '.join(x.astype(str)), axis=1)

        # Usual boxplot for each variable that was benchmarked
        for plotVar in boxPlotsToMake:

            # make it wider for the insanse length of xticklabels
            fig = plt.figure(figsize=(10, 5))

            plt.style.use("seaborn-darkgrid")
            bp = sns.boxplot(data=benchMarkingDF,
                             x=mergedParamsName, y=plotVar)
            sns.swarmplot(x=mergedParamsName, y=plotVar, data=benchMarkingDF,
                          color="grey")

            # for readability of axis labels
            bp.set_xticklabels(bp.get_xticklabels(), rotation=45, ha='right')

            newPlotTitle = plotVar + '-' + plotTitle
            plt.title('Benchmarking of Sampled Planner ' + plotVar)
            savePlot(fig=fig, shouldSavePlots=self.shouldSavePlots,
                     baseSaveFName=baseSaveFName, plotTitle=newPlotTitle)

        ##
        # Plotting path validity bar graph
        ##

        # number of times a valid path was found
        fig = plt.figure()

        plt.style.use('seaborn-darkgrid')
        bp = sns.barplot(x=mergedParamsName, y='numValidPaths',
                         data=pathValidityDF)
        plt.title('Number of Valid Paths Found for Each Parameter Combination')

        # for readability of axis labels
        bp.set_xticklabels(bp.get_xticklabels(), rotation=45, ha='right')

        newPlotTitle = 'numPaths' + '-' + plotTitle
        savePlot(fig=fig, shouldSavePlots=self.shouldSavePlots,
                 baseSaveFName=baseSaveFName, plotTitle=newPlotTitle)

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

        # need to load the config data here to extract simulation benchmarking
        # parameters
        configData = Builder.loadConfigData(configFileName)
        nodes = configData['nodes']
        adjList = configData['edges']
        startNode = configData['startNodeLabel']
        goalNode = configData['goalNodeLabel']

        # need to convert the configuration adjacency list given in the config
        # to an edge list given as a 3-tuple of (source, dest, edgeAttrDict)
        edgeList = []
        for sourceEdge, destEdgesData in adjList.items():
            if destEdgesData:
                newEdges = [(sourceEdge, destEdgeData[0], destEdgeData[1]) for
                            destEdgeData in destEdgesData]
                edgeList.extend(newEdges)

        myLittleGraph = Graph(nodes, edges=edgeList)
        myLittleGraph.dispEdges()

        # make a generic solution info printing function
        def printShit(method, pathLength, numIter):
            print('|--------  ', method, '  -------|')
            if pathLength:
                print('Path Length:', pathLength)
            else:
                print('No path exists')
            print('Number of Dequeues to Find Path:', numIter)

        # define a generic path printing function
        def plotPath(method, graph, path, pathLength, shouldSavePlots,
                     baseSaveFName):
            if path:
                plotTitle = 'Shortest Path (length = ' + pathLength + \
                            ') Found with ' + method
            else:
                plotTitle = 'No Path Found with ' + method

            fig = graph.plot(path, plotTitle)
            savePlot(fig=fig, shouldSavePlots=shouldSavePlots,
                     baseSaveFName=baseSaveFName, plotTitle=plotTitle)

        # now run and compare the performance statistics of A* to plain
        # Dijkstra's
        path = {}
        pathLength = {}
        numIter = {}

        ##
        # A* Baby
        ##
        method = 'A*'
        (path[method],
         pathLength[method],
         numIter[method]) = myLittleGraph.findPathToGoal(start=startNode,
                                                         goal=goalNode,
                                                         method=method)
        printShit(method, pathLength[method], numIter[method])
        plotPath(method, myLittleGraph, path[method], pathLength[method],
                 self.shouldSavePlots, baseSaveFName)

        ##
        # Dijkstra :'(
        ##
        method = 'Dijkstra'
        (path[method],
         pathLength[method],
         numIter[method]) = myLittleGraph.findPathToGoal(start=startNode,
                                                         goal=goalNode,
                                                         method=method)
        printShit(method, pathLength[method], numIter[method])
        plotPath(method, myLittleGraph, path[method], pathLength[method],
                 self.shouldSavePlots, baseSaveFName)

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
            configNames = ['graph1']

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
