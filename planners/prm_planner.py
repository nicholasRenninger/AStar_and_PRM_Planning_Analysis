# 3rd-party packages
import matplotlib.pyplot as plt
from timeit import default_timer as timer
import numpy as np
from scipy import spatial
from networkx.utils import UnionFind
import random
import copy

# local packages
from planners.planner import Planner
from factory.builder import Builder
from spaces.graph import Graph


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

        # load in planner settings from the config file
        self.n = configData['n']
        self.r = configData['r']
        self.usePathSmoothing = configData['smoothing']
        self.minCSpaceBounds = configData['minCSpaceBounds']
        self.maxCSpaceBounds = configData['maxCSpaceBounds']
        self.numSmoothingIters = 1000

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
            n = self.n[0]
            r = self.r[0]
            usePathSmoothing = self.usePathSmoothing[0]

        start = timer()
        (graph,
         shortestPath,
         pathLength, foundPath) = self.computePRM(startState, goalState, n, r,
                                                  usePathSmoothing)
        finish = timer()
        computationTime = finish - start

        # plot the resulting path over the PRM computation
        shouldPlot = plotConfigData['shouldPlot']

        if shouldPlot:
            title = 'PRM - path length = %0.3g  n = %d  r = %0.3g' \
                % (self.robot.distTraveled, n, r)
            plotConfigData['plotTitle'] += title
            self.plot(graph, startState, goalState, plotConfigData,
                      path=shortestPath)

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

        # compute all samples at once, no need to check for collision
        samples = self.getSamples(numSamples=n,
                                  lowerBounds=self.minCSpaceBounds,
                                  upperBounds=self.maxCSpaceBounds)

        # put them in a K-D tree to allow for easy connectivity queries
        kdTree = spatial.cKDTree(samples)

        # add all start, goal, and sampled nodes
        graph = Graph()
        (graph,
         startNodeLabel,
         goalNodeLabel) = self.addAllNodesToGraph(graph, samples,
                                                  startState, goalState, n)

        # now connect all of the samples within radius r of each other
        graph = self.connectAllNodesInRadius(graph, kdTree, r)

        print('starting search')
        (shortestPath,
         pathLength, _) = graph.findPathToGoal(startNodeLabel,
                                               goalNodeLabel,
                                               method='A star')
        foundPath = (shortestPath is not None)

        # only start smoothing if desired
        if foundPath and usePathSmoothing:
            print('smoothing path')
            (shortestPath,
             pathLength) = self.smoothPathInGraph(graph, shortestPath,
                                                  goalNodeLabel, pathLength)

        # run robot through whole path
        if foundPath:
            for node in shortestPath:

                currPos = graph.getNodeData(node, 'pos')
                self.robot.updateRobotState(currPos)

        return (graph, shortestPath, pathLength, foundPath)

    ##
    # @brief      Computes numSamples samples from a multivariate uniform
    #             distribution given by its upper and lower bounds
    #
    # @param      numSamples   number of samples to compute
    # @param      lowerBounds  A length m list of lower bounds in each of the m
    #                          coordinate directions
    # @param      upperBounds  A length m list of upper bounds in each of the m
    #                          coordinate directions
    #
    # @return     A (numSamples, m) numpy array of uniform samples
    #
    def getSamples(self, numSamples, lowerBounds, upperBounds):

        dimensionalityOfCSpace = len(lowerBounds)

        samples = np.random.uniform(low=lowerBounds, high=upperBounds,
                                    size=(numSamples, dimensionalityOfCSpace))

        return samples

    #
    # @brief      Gets a sample in Cspace that has no collision on the line
    #             between start and the new sampled node
    #
    # @param      startNodePos  The start node's position
    # @param      lowerBounds   A length m list of lower bounds in each of the
    #                           m coordinate directions
    # @param      upperBounds   A length m list of upper bounds in each of the
    #                           m coordinate directions
    #
    # @return     The collision free sample's position
    #
    def getCollisionFreeSample(self, startNodePos, endNodePos, lowerBounds,
                               upperBounds):

        dimensionalityOfCSpace = len(lowerBounds)
        npSampleSize = (1, dimensionalityOfCSpace)

        collided = True
        iterNum = 0

        # if we hit max iterations and we're still collided, need to
        # return this info so we don't use the sample
        foundGoodSample = False

        while collided and iterNum <= self.numSmoothingIters:

            potentialSample = np.random.uniform(low=lowerBounds,
                                                high=upperBounds,
                                                size=npSampleSize)
            potentialSample = potentialSample.flatten()

            # need to check that the whole new sample path won't collide
            collidedTo = self.checkCollision(startNodePos, potentialSample)
            collidedFrom = self.checkCollision(potentialSample, endNodePos)
            collided = collidedTo or collidedFrom

            iterNum += 1

        # Yay! we found a good sample!
        if not collided:
            foundGoodSample = True

        return foundGoodSample, potentialSample

    ##
    # @brief      Adds all nodes to the graph instance
    #
    # @param      graph       The Graph instance to add nodes to
    # @param      samples     The numpy array of sampled node positions to add
    # @param      startState  The start state position
    # @param      goalState   The goal state position
    # @param      n           number of samples
    #
    # @return     (graph with all start, goal, and sampled nodes added,
    #              the graph label of the startState,
    #              the graph label of the goalState)
    #
    def addAllNodesToGraph(self, graph, samples, startState, goalState, n):

        startNodeLabel = n + 1
        graph.add_node(startNodeLabel,
                       heuristicDist=self.calculateDist(startState, goalState),
                       prev=None, dist=0, priority=0, pos=startState.flatten())

        goalNodeLabel = n + 2
        graph.add_node(goalNodeLabel,
                       heuristicDist=0, prev=None, dist=np.inf,
                       priority=np.inf, pos=goalState.flatten())

        # now initialize the sampled nodes of the underlying PRM graph
        for sampleNodeLabel in range(0, n):

            pos = samples[sampleNodeLabel, :]
            heuristicDist = self.calculateDist(pos, goalState)
            graph.add_node(sampleNodeLabel,
                           heuristicDist=heuristicDist,
                           prev=None, dist=np.inf,
                           priority=np.inf, pos=pos.flatten())

        return (graph, startNodeLabel, goalNodeLabel)

    ##
    # @brief      Connects all nodes within radius of each other in CSpace
    #
    # @param      graph   The initialized Graph object with all nodes added
    # @param      kdTree  The initialized KDTree with all values stored, used
    #                     for find
    # @param      radius  The radius to connect samples by
    #
    # @return     the PRM Graph with all radius-neighbored nodes connected as
    #             weighted Graph edges
    #
    def connectAllNodesInRadius(self, graph, kdTree, radius):

        # keep a union-find data structure to improve search performance by not
        # allowing cycles in the graph
        graphPartitions = UnionFind()

        for currNodeLabel, currNodeData in list(graph.nodes(data=True)):

            currPos = currNodeData['pos']

            # search for all nodes in radius of the current node in question
            pointToCheck = currPos.flatten()
            neighborLabelsInRadius = kdTree.query_ball_point(pointToCheck,
                                                             radius)

            # adding all NEW edges that don't collide to the graph
            for neighborLabel in neighborLabelsInRadius:

                goalPos = graph.getNodeData(neighborLabel, 'pos')

                collides = self.checkCollision(currPos, goalPos)
                notInSameComponent = self.checkConnectivity(graphPartitions,
                                                            currNodeLabel,
                                                            neighborLabel)

                if (not collides) and notInSameComponent:

                    weight = self.calculateDist(currPos, goalPos)
                    graph.add_edge(currNodeLabel, neighborLabel,
                                   weight=weight)

                    # need to update union-find data with the new edge
                    graphPartitions.union(currNodeLabel, neighborLabel)

        return graph

    ##
    # @brief      checks edge collision by checking if the path between the
    #             nodes will collide with anything in Cspace
    #
    # @param      startPosition  The start node's position
    # @param      goalPos        The goal node's position
    #
    # @return     bool, whether or not the line between start and goal collides
    #             with any obstacle
    #
    def checkCollision(self, startPosition, goalPos):

        collides = self.robot.checkLinearPathCollision(startPosition, goalPos)

        return collides

    ##
    # @brief      check if the two nodes are part of the same component already
    #
    # @param      unionFindADT   The union find adt storing graph components
    # @param      currNodeLabel  The current node's Graph label
    # @param      neighborLabel  The neighbor node's Graph label
    #
    # @return     bool, whether or not the curr and neighbor are in the same
    #             component
    #
    def checkConnectivity(self, unionFindADT, currNodeLabel, neighborLabel):

        currComponent = unionFindADT[currNodeLabel]
        newComponent = unionFindADT[neighborLabel]

        nodesAreNotInSameComponent = (currComponent != newComponent)
        return nodesAreNotInSameComponent

    ##
    # @brief      smooths path in the already computed PRM path
    #
    # @param      graph          The connected PRM Graph
    # @param      path           The list of node labels describing the path to
    #                            smooth
    # @param      goalNodeLabel  The goal node's label
    # @param      pathLength     The path length of the unsmoothed path
    #
    # @return     graph will have more valid paths that are hopefully
    #             "smoother"
    #
    def smoothPathInGraph(self, graph, path, goalNodeLabel, pathLength):

        newPath = copy.deepcopy(path)

        numEdgesToSmooth = round(len(newPath) / 5)

        for i in range(0, numEdgesToSmooth):

            # only allow sampling from the middle of the path
            safePath = newPath[1:-1]
            rNodes = tuple(self.orderedSampleWithoutReplacement(safePath, 2))
            startNodeLabel = rNodes[0]
            endNodeLabel = rNodes[1]

            # skip the sampled nodes if they're already directly connected
            nodeBeforeEnd = graph.getNodeData(endNodeLabel, 'prev')
            if nodeBeforeEnd == startNodeLabel:
                continue

            # obtain the collision free samples
            startNodePos = graph.getNodeData(startNodeLabel, 'pos')
            endNodePos = graph.getNodeData(endNodeLabel, 'pos')
            (foundGoodSample,
             samplePos) = self.getCollisionFreeSample(startNodePos,
                                                      endNodePos,
                                                      self.minCSpaceBounds,
                                                      self.maxCSpaceBounds)
            # check if the sample is None. if its None, we couldn't find a
            # sample, so just move on
            if not foundGoodSample:
                continue

            # add the node to the PRM graph
            newNodeLabel = goalNodeLabel + i + 1
            goalNodePos = graph.getNodeData(goalNodeLabel, 'pos')
            newHeuristicDist = self.calculateDist(samplePos, goalNodePos)
            graph.add_node(newNodeLabel,
                           heuristicDist=newHeuristicDist,
                           prev=startNodeLabel,
                           dist=0, priority=0, pos=samplePos)

            # connect it to the graph
            distStart = self.calculateDist(startNodePos, samplePos)
            graph.add_edge(startNodeLabel, newNodeLabel, weight=distStart)

            distEnd = self.calculateDist(samplePos, endNodePos)
            graph.add_edge(newNodeLabel, endNodeLabel, weight=distEnd)

            # remove in-between nodes on the path
            currNode = endNodeLabel
            prevNode = graph.getNodeData(currNode, 'prev')

            while prevNode != startNodeLabel:

                prevPrevNode = graph.getNodeData(prevNode, 'prev')
                newPath.remove(prevNode)

                # need to update prev now in order to continue proper traversal
                graph.setNodeData(currNode, 'prev', prevPrevNode)

                # now set the linked list pointers
                prevNode = graph.getNodeData(prevNode, 'prev')

            # now insert the new node into its place
            endNodeIDX = newPath.index(endNodeLabel)
            newPath.insert(endNodeIDX, newNodeLabel)
            graph.setNodeData(endNodeLabel, 'prev', newNodeLabel)

        # compute new path length
        newPathEdges = graph.getPathEdges(newPath)
        newPathLength = 0
        for edge in newPathEdges:
            newPathLength += graph.edges[edge]['weight']

        # only return the smoothed path if its shorter
        if newPathLength > pathLength:
            print('smoothing failed, using unsmoothed path')
            return path, pathLength
        else:
            return newPath, newPathLength

    ##
    # @brief      Calculate the point's distance to another point
    #
    # @param      point     The point location np array
    # @param      dest      The destination location np array
    # @param      distNorm  optional np.linalg.norm norm ord parameter. default
    #                       is 2
    #
    # @return     The distance to dest.
    #
    def calculateDist(self, point, dest, distNorm=2):

        point_copy = np.reshape(point, (len(point), 1))
        dest_copy = np.reshape(dest, (len(dest), 1))

        distance = np.linalg.norm(dest_copy - point_copy, ord=distNorm)

        return distance

    #
    # @brief      Take a random sample without replacement of the indices, sort
    #             the indices, and take them from the original.
    #
    # Optimized O(N)-time, O(1)-auxiliary-space
    #
    # shamelessly from
    # https://stackoverflow.com/a/6482925
    #
    # @param      seq         The iterable object to sample from
    # @param      sampleSize  The number of samples to return
    #
    # @return     a sequence of sampleSize samples from seq, such that each
    #             sequential sample appears before the next in seq
    #
    def orderedSampleWithoutReplacement(self, seq, sampleSize):

        totalElems = len(seq)
        if not 0 <= sampleSize <= totalElems:
            raise ValueError('Required that 0 <= sample_size' +
                             '<= population_size')

        picksRemaining = sampleSize
        for elemsSeen, element in enumerate(seq):
            elemsRemaining = totalElems - elemsSeen
            prob = picksRemaining / elemsRemaining
            if random.random() < prob:
                yield element
                picksRemaining -= 1

    ##
    # @brief      plots the PRM distance computation overlaid on the cspace
    #             plot
    #
    # @param      graph           The networkx object graph object computed by
    #                             the PRM algorithm
    # @param      startState      The start config state
    # @param      goalState       The goal config state
    # @param      plotConfigData  The plot configuration data
    # @param      path            The list of node labels defining a path
    #
    def plot(self, graph, startState, goalState,
             plotConfigData, path=None):

        fig = plt.figure()
        ax = fig.add_subplot(111)

        # plot the graph and its shortest path
        fig, ax = graph.plot(path=path, fig=fig, showLabels=False,
                             showEdgeWeights=False)

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
