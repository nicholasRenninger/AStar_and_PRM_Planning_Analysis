# 3rd-party packages
import matplotlib.pyplot as plt
import networkx as nx
import heapq
import queue
import numpy as np
import copy


##
# @brief      This class wraps a networkx graph implementation
#
class Graph(nx.Graph):

    ##
    # @brief      Constructs a new instance of the Graph
    #
    # @param      nodes  A dict of node label key and then node attribute
    #                    dictionary value such that nodes can be directly added
    #                    to a networkx graph
    # @param      edges  A list of edge dicts that can be directly added to a
    #                    networkx graph
    #
    def __init__(self, nodes=[], edges=[]):

        # need to start with a fully initialized networkx digraph
        super().__init__()

        if nodes:
            self.add_nodes_from(nodes)

            if edges:
                self.add_edges_from(edges)

            # save all of the node attributes
            # @warning assuming here that each node has the same attributes
            # currently:
            #   - 'heuristicDist'
            #   - 'prev'
            #   - 'dist'
            #   - 'priority'
            #   - 'pos'
            self.nodeProperties = set([k for n in self.nodes
                                       for k in self.nodes[n].keys()])

    ##
    # @brief      Finds a path in the graph from start to goal using the search
    #             algorithm specified by method string
    #
    # @param      start   The start node label
    # @param      goal    The goal node label
    # @param      method  The method string:
    #                       - 'A star'
    #                       - 'Dijkstra'
    #
    # @return     (path as a list of node labels in the graph - None if no path
    #             found, length of the shortest path - None if no path found,
    #             number of iterations of the while loop (num of node dequeues)
    #             needed to find a path)
    #
    def findPathToGoal(self, start, goal, method):

        path = None
        pathLength = None
        numIter = 0

        if start == goal:
            path = [start]
            return (path, pathLength, numIter)

        # initialize the auxillary distance and tree arrays
        self.reset()
        self.setNodeData(start, 'priority', 0)
        self.setNodeData(start, 'dist', 0)

        # both A star and dijkstra need a set type (unique element) priority
        # queue
        Q = UniquePriorityQueue()

        # start the search at the start node
        Q.put(self.getPriorityTuple(start))

        # iterate until we cant add any more nodes or we reach the goal node
        while not Q.empty():

            # pop the minimum priority node off of the queue
            currPriority, currNode = Q.get()
            numIter += 1

            # determine whether to stop searching based on the search
            # method
            if self.searchShouldHalt(currNode, goal, method):

                pathLength = self.getNodeData(currNode, 'dist')
                path = self.reversePathFromGoal(start, goal)
                break

            # need to examine all outgoing edges and relax them
            for neighbor in self.adj[currNode]:

                currEdge = (currNode, neighbor)

                if self.isTense(currEdge, method):

                    # once we find a tense node, relax it and add it to the
                    # search priority queue
                    self.relax(currEdge, method)
                    Q.put(self.getPriorityTuple(neighbor))

        return (path, pathLength, numIter)

    ##
    # @brief      Determines if an edge is "tense" - not part of the solution
    #             as there exists an alternate path with smaller total expected
    #             weight
    #
    # @param      edgeLabel  The edge tuple: (src label, dest label)
    # @param      method     The method string:
    #                       - 'A star'
    #                       - 'Dijkstra'
    #
    # @return     True if tense, False otherwise.
    #
    def isTense(self, edgeLabel, method):

        (source, dest, weight, srcDist, _,
         _, destPriority) = self.getEdgeData(edgeLabel, method)

        # need to prevent cyclical paths when using undirected graphs
        alreadyVisitedEdge = (self.getNodeData(source, 'prev') == dest)
        if not alreadyVisitedEdge:
            edgeIsTense = (srcDist + weight) < destPriority
        else:
            edgeIsTense = False

        return edgeIsTense

    ##
    # @brief      "Relaxes a node" according to the method string, adding the
    #             node to the final search solution
    #
    # @param      edgeLabel  The edge tuple: (src label, dest label)
    # @param      method     The method string:
    #                       - 'A star'
    #                       - 'Dijkstra'
    #
    # @return     the priority of the destination node (i.e. distance)
    #
    def relax(self, edgeLabel, method):

        (source, dest, weight,
         _, _, _, _) = self.getEdgeData(edgeLabel, method)
        destDist, destPriority = self.calcDestNodeDist(source, dest,
                                                       weight, method)

        self.setNodeData(dest, 'dist', destDist)
        self.setNodeData(dest, 'priority', destPriority)

        # need to prevent cyclical paths when using undirected graphs
        alreadyVisitedEdge = (self.getNodeData(source, 'prev') == dest)
        if not alreadyVisitedEdge:
            self.setNodeData(dest, 'prev', source)

        return destDist

    ##
    # @brief      Returns the graph distance metric to the destination node
    #             given the source node, the edge weight and method
    #
    # @param      source  The source node label
    # @param      dest    The destination node label
    # @param      weight  The edge weight label
    # @param      method  The search method string:
    #                     - 'Dijkstra': just use node's "dist" property
    #                     - 'A star': add the node's "dist" property to its
    #                       "heuristicDist"
    #
    # @return     (The dest node's distance metric under method., its
    #             "priority" which can include heuristic distance measures)
    #
    def calcDestNodeDist(self, source, dest, weight, method):

        if method == 'A star':

            destDist = copy.deepcopy(self.getNodeData(source, 'dist')) + weight
            destPriority = copy.deepcopy(destDist) + \
                self.getNodeData(dest, 'heuristicDist')

        elif method == 'Dijkstra':

            destDist = copy.deepcopy(self.getNodeData(source, 'dist')) + weight
            destPriority = copy.deepcopy(destDist)

        else:
            raise ValueError(method)

        return (destDist, destPriority)

    ##
    # @brief      Gets all edge data needed for edge relaxation algorithms
    #
    # @param      edgeLabel  The edge tuple: (src label, dest label)
    # @param      method     The method string:
    #                       - 'A star'
    #                       - 'Dijkstra'
    #
    # @return     (source node label, destination node label, edge weight,
    #             distance to source node under method distance metric, source
    #             node "priority" which can include heuristic distance
    #             measures, distance to destination node under method distance
    #             metric, destination node "priority" which can include
    #             heuristic distance measures)
    #
    def getEdgeData(self, edgeLabel, method):

        source = edgeLabel[0]
        dest = edgeLabel[1]
        weight = self.edges[edgeLabel]['weight']

        srcDist = self.getNodeData(source, 'dist')
        srcPriority = self.getNodeData(source, 'priority')

        destDist = self.getNodeData(dest, 'dist')
        destPriority = self.getNodeData(dest, 'priority')

        return (source, dest, weight, srcDist, srcPriority,
                destDist, destPriority)

    ##
    # @brief      Determines whether the search should halt
    #
    # @param      currNode  The current node label
    # @param      goal      The goal node label
    # @param      method    The method string:
    #                       - 'A star'
    #                       - 'Dijkstra'
    #
    # @return     boolean as to whether the search should stop
    #
    def searchShouldHalt(self, currNode, goal, method):

        atGoal = (currNode == goal)

        if method == 'A star':

            if atGoal:

                prevNode = self.getNodeData(currNode, 'prev')
                currDist = self.getNodeData(currNode, 'dist')
                prevPriority = self.getNodeData(prevNode, 'priority')

                noShorterPathsStillToSearch = (currDist <= prevPriority)

                return noShorterPathsStillToSearch

        elif method == 'Dijkstra':

            return atGoal

        else:
            raise ValueError(method)

    ##
    # @brief      Finds the path from start to goal after a search algorithm by
    #             following prev node pointers backwards through the graph from
    #             the goal
    #
    # @param      start  The start node label
    # @param      goal   The goal node label
    #
    # @return     list of node labels starting with the start node
    #
    def reversePathFromGoal(self, start, goal):

        currNode = goal
        path = [currNode]

        while currNode != start:

            currNode = self.getNodeData(currNode, 'prev')
            path.append(currNode)

        # have to reverse the path in place as we traversed backwards
        path.reverse()

        return path

    ##
    # @brief      Gets a list of edges from the path list
    #
    # @param      path  The path list
    #
    # @return     a list of path edges as node label tuples
    #
    def getPathEdges(self, path):

        path_edges = [(v1, v2) for v1, v2 in zip(path, path[1:])]

        return path_edges

    ##
    # @brief      Plots the networkx graph and any given paths through the
    #             graph
    #
    # @param      path             A list of lists of Nodes defining each path
    # @param      fig              The mpl figure handle to plot the graph on
    # @param      plotTitle        The plot title string
    # @param      baseSize         The base size of each node, gets scaled by
    #                              the length of each node
    # @param      node_size        The node size
    # @param      showLabels       flag to show node labels
    # @param      showEdgeWeights  flag to show edge weights
    # @param      showAxes         flag to turn on / off the axes
    #
    # @return     (figure handles to the graph, axes handle to the graph)
    #
    def plot(self, path=None, fig=None, plotTitle=None,
             baseSize=400, node_size=10, showLabels=True,
             showEdgeWeights=True, showAxes=True):

        if not fig:
            fig = plt.figure()

        # scale node sizes by string length only if all node labels are strings
        allStrs = bool(self.nodes()) and all(isinstance(elem, str)
                                             for elem in self.nodes())
        pos = nx.get_node_attributes(self, 'pos')
        if allStrs:
            node_size = [len(v) * baseSize for v in self.nodes()]
            nx.draw_networkx(self, pos=pos,
                             with_labels=showLabels, node_size=node_size)
        else:
            nx.draw_networkx(self, pos=pos, with_labels=showLabels,
                             node_size=node_size,
                             cmap=plt.get_cmap('jet'))

        # show edge weights as well
        if showEdgeWeights:
            labels = nx.get_edge_attributes(self, 'weight')
            nx.draw_networkx_edge_labels(self, pos, edge_labels=labels)

        # draw path through the graph if it exists
        if path:

            nx.draw_networkx_nodes(self, pos, nodelist=path, node_color='r',
                                   node_size=node_size)

            path_edges = self.getPathEdges(path)
            nx.draw_networkx_edges(self, pos, edgelist=path_edges,
                                   edge_color='r', width=4)

        # Axes settings
        ax = plt.gca()
        ax.set_title(plotTitle)
        if showAxes:
            ax.tick_params(left=True, bottom=True,
                           labelleft=True, labelbottom=True)
        else:
            [sp.set_visible(False) for sp in ax.spines.values()]
            ax.set_xticks([])
            ax.set_yticks([])

        return (fig, ax)

    ##
    # @brief      Returns a tuple with the node's priority and label for use in
    #             the priority queue
    #
    # @param      node  The node label
    #
    # @return     The (priority, node label) tuple for node
    #
    def getPriorityTuple(self, node):

        return (self.getNodeData(node, 'priority'), node)

    ##
    # @brief      Gets the node's dataKey data from the graph
    #
    # @param      nodeLabel  The node label
    # @param      dataKey    The data key string
    #
    # @return     The node data associated with the nodeLabel and dataKey
    #
    def getNodeData(self, nodeLabel, dataKey):

        nodeData = self.nodes.data()

        return nodeData[nodeLabel][dataKey]

    ##
    # @brief      Sets the node's dataKey data from the graph
    #
    # @param      nodeLabel  The node label
    # @param      dataKey    The data key string
    # @param      data       The data to set the item at dataKey to
    #
    def setNodeData(self, nodeLabel, dataKey, data):

        nodeData = self.nodes.data()
        nodeData[nodeLabel][dataKey] = data

    ##
    # @brief      prints the graph as a sequence of weighted edges
    #
    def dispEdges(self):

        for n, nbrs in self.adj.items():
            for nbr, eattr in nbrs.items():

                wt = eattr['weight']
                print('(%s, %s, %0.3g)' % (str(n), str(nbr), wt))

    ##
    # @brief      prints each node's data
    #
    def dispNodes(self):

        for node in self.nodes(data=True):
            print(node)

    ##
    # @brief      Resets the graph's nodes to default state in order to run
    #             search algorithms on
    #
    def reset(self):

        for node in self.nodes:

            self.setNodeData(node, 'dist', np.inf)
            self.setNodeData(node, 'priority', np.inf)
            self.setNodeData(node, 'prev', None)


##
# @brief      Implements a priority queue that maintains unique elements
#
#             Implemented according to official heapq documentation:
#             https://stackoverflow.com/questions/5997189/how-can-i-make-a-unique-value-priority-queue-in-python
#
# - https://github.com/python/cpython/blob/2.7/Lib/Queue.py
# - https://docs.python.org/3/library/heapq.html
#
class UniquePriorityQueue(queue.Queue):

    def _init(self, maxsize):
        """!
        @brief      Initializes the given maxsize.
        @param      maxsize  The maxsize
        """

        self.queue = []
        self.REMOVED = '<removed-task>'
        self.entry_finder = {}

    def _put(self, item, heappush=heapq.heappush):
        """!
        @brief      adds the tuple of priority and item to the queue
        @param      item      The item
        @param      heappush  The heappush
        """

        item = list(item)
        priority, task = item

        if task in self.entry_finder:
            # Do not add new item.
            pass
        else:
            self.entry_finder[task] = item
            heappush(self.queue, item)

    def _qsize(self, len=len):

        return len(self.entry_finder)

    def is_empty(self):
        """!
        Determines if the priority queue has any elements. Performs removal of
        any elements that were "marked-as-invalid".

        :returns: true iff the priority queue has no elements.

        @return     True if empty, False otherwise.
        """
        while self.pq:
            if self.queue[0][1] != self.REMOVED:
                return False
            else:
                _, _, element = heapq.heappop(self.pq)
                if element in self.element_finder:
                    del self.element_finder[element]
        return True

    def _get(self, heappop=heapq.heappop):
        """
        The base makes sure this shouldn't be called if `_qsize` is 0.

        @param      heappop  The heappop

        @return     the lowest priority item
        """
        while self.queue:

            item = heappop(self.queue)
            _, task = item

            if task is not self.REMOVED:

                del self.entry_finder[task]
                return item

        raise KeyError('should never happen: pop from an empty priority queue')
