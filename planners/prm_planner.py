# 3rd-party packages
import matplotlib.pyplot as plt
import networkx as nx
from timeit import default_timer as timer

# local packages
from planners.planner import Planner
from factory.builder import Builder


##
# @brief      This class describes a Planner subclass used to find a path in
#             two-dimensional configuration space using a probabilistic roadmap
#             approach
#
class PRMPlanner(Planner):

    ##
    # @brief      PRMPlanner class constructor
    #
    # @param      plannerType      The planner type string
    # @param      cSpace           The configuration space of the robot
    # @param      workspace        The workspace object the robot operates in
    # @param      robot            The Planner type string
    # @param      configData       Configuration dictionary for the planner
    # @param      shouldSavePlots  Boolean controlling whether or not the plt
    #                              objs can be saved to the baseSaveName dir
    # @param      baseSaveFName    The base directory file name for output plot
    #
    # @return     initialized Planner object
    #
    def __init__(self, plannerType, cSpace, workspace, robot, configData,
                 shouldSavePlots, baseSaveFName):

        # calling the superclass contructor to inherit its properties
        Planner.__init__(self,
                         plannerType=plannerType,
                         cSpace=cSpace,
                         workspace=workspace,
                         robot=robot,
                         configData=configData,
                         shouldSavePlots=shouldSavePlots,
                         baseSaveFName=baseSaveFName)

        # load in planner settings from the confg file
        self.n = configData['n']
        self.r = configData['r']
        self.usePathSmoothing = configData['smoothing']

    ##
    # @brief      Computes a viable path in robot's cSpace to the goalState
    #             from the robot's startState
    #
    # @param      startState         The start config state
    # @param      goalState          The goal config state
    # @param      plannerConfigData  A dictionary containing planner
    #                                configuration data, None if using internal
    #                                config data:
    #                                   - 'n': Fixed number of samples to try
    #                                   - 'r': the radius in cspace in which to
    #                                     try to connect samples together
    # @param      plotConfigData     The plot config dictionary
    #
    # @return     a viable set of cSpace states from startState to goalState
    #             (the)
    #
    def findPathToGoal(self, startState, goalState, plannerConfigData,
                       plotConfigData):

        # allow the user to overide the settings in the config file
        if plannerConfigData:

            n = plannerConfigData['n']
            r = plannerConfigData['r']
            usePathSmoothing = plannerConfigData['smoothing']

        else:
            n = self.n
            r = self.r
            usePathSmoothing = self.usePathSmoothing

        start = timer()
        (graph,
         shortestPath,
         foundPath) = self.computePRM(startState, goalState, n, r,
                                      usePathSmoothing)
        finish = timer()
        computationTime = finish - start

        # plot the resulting path over the PRM computation
        shouldPlot = plotConfigData['shouldPlot']

        if shouldPlot:
            title = 'PRM: path length = %0.3g' % self.robot.distTraveled
            plotConfigData['plotTitle'] += title
            self.plot(graph, shortestPath,
                      startState, goalState, plotConfigData)

        return (computationTime, foundPath)

    ##
    # @brief      Implements the PRM path finding algorithm
    #
    # @param      startState        The start config state
    # @param      goalState         The goal config state
    # @param      n                 Fixed number of samples to try
    # @param      r                 the radius in cspace in which to try to
    #                               connect samples together
    # @param      usePathSmoothing  Flag to enable / disable path smoothing
    #
    # @return     (the networkx graph object computed by the prm algortihm, the
    #             shortest path found by the search on the connected PRM,
    #             whether or not the planner found a path, self.robot has its
    #             state updated according to the best path found by the
    #             planner)
    #
    def computePRM(self, startState, goalState, n, r, usePathSmoothing):

        foundPath = False
        graph = None
        shortestPath = None

        return (graph, shortestPath, foundPath)

    ##
    # @brief      plots the PRM distance computation overlaid on the cspace
    #             plot
    #
    # @param      graph           The networkx object graph object computed by
    #                             the PRM algorithm
    # @param      shortestPath    The networkx shortest path object computed by
    #                             the PRM algorithm
    # @param      startState      The start config state
    # @param      goalState       The goal config state
    # @param      plotConfigData  The plot configuration data
    # @param      distCells  The distance cells with PRM distance computed
    #
    def plot(self, graph, shortestPath, startState, goalState, plotConfigData):

        fig = plt.figure()
        ax = fig.add_subplot(111)

        # plot the graph and its shortest path
        G = nx.karate_club_graph()
        pos = nx.spring_layout(G)
        nx.draw(G, pos, node_color='k')

        # draw path in red
        path = nx.shortest_path(G, source=14, target=16)
        path_edges = [(v1, v2) for v1, v2 in zip(path, path[1:])]

        nx.draw_networkx_nodes(G, pos, nodelist=path, node_color='r')
        nx.draw_networkx_edges(G, pos,
                               edgelist=path_edges, edge_color='r',
                               width=4)

        self.cSpace.plot(robot=self.robot,
                         startState=startState,
                         goalState=goalState,
                         plotConfigData=plotConfigData,
                         ax=ax, fig=fig)


##
# @brief      Implements the generic builder class for the PRMPlanner
#
class PRMPlannerBuilder(Builder):

    ##
    # @brief      need to call the super class constructor to gain its
    #             properties
    #
    def __init__(self):

        Builder.__init__(self)
        self.robot = None
        self.workspace = None

    ##
    # @brief      Implements the smart constructor for PRMPlanner
    #
    #             Only reads the config data once, otherwise just returns the
    #             built object
    #
    # @param      plannerType      The planner type string
    # @param      cSpace           The configuration space of the robot
    # @param      workspace        The workspace object the robot operates in
    # @param      robot            The Planner type string
    # @param      configFileName   The configuration file name
    # @param      shouldSavePlots  Boolean controlling whether or not the plt
    #                              objs can be saved to the baseSaveName dir
    # @param      baseSaveFName    The base directory file name for output plot
    #
    # @return     instance of an initialized PRMPlanner object
    #
    def __call__(self, plannerType, cSpace, workspace, robot, configFileName,
                 shouldSavePlots, baseSaveFName):

        robotHasChanged = (self.robot != robot)
        workspaceHasChanged = (self.workspace != workspace)
        configNameHasChanged = (self._configName != configFileName)
        noInstanceLoadedYet = (self._instance is None)

        if noInstanceLoadedYet or robotHasChanged or workspaceHasChanged or \
           configNameHasChanged:

            configData = self.loadConfigData(configFileName)
            self._instance = PRMPlanner(plannerType=plannerType,
                                        cSpace=cSpace,
                                        workspace=workspace,
                                        robot=robot,
                                        configData=configData,
                                        shouldSavePlots=shouldSavePlots,
                                        baseSaveFName=baseSaveFName)

        return self._instance
