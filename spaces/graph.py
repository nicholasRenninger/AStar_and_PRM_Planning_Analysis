# 3rd-party packages
import matplotlib.pyplot as plt
import networkx as nx


##
# @brief      This class wraps a networkx graph implementation
#
class Graph(nx.DiGraph):

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

        self.add_nodes_from(nodes)
        self.add_edges_from(edges)

    ##
    # @brief      Finds a path in the graph from start to goal using the search
    #             algorithm specified by method string
    #
    # @param      start   The start node label string
    # @param      goal    The goal node label string
    # @param      method  The method string:
    #                       - 'A*'
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

        return (path, pathLength, numIter)

    ##
    # @brief      Plots the networkx graph and any given paths through the
    #             graph
    #
    # @param      path       A list of lists of Nodes defining each path
    # @param      fig        The mpl figure handle to plot the graph on
    # @param      plotTitle  The plot title string
    #
    # @return     figure handles to the graph
    #
    def plot(self, path=None, fig=None, plotTitle='MyLittlePony'):

        if not fig:
            fig = plt.figure()

        return fig

    ##
    # @brief      prints the graph as a sequence of weighted edges
    #
    def dispEdges(self):

        for n, nbrs in self.adj.items():
            for nbr, eattr in nbrs.items():

                wt = eattr['weight']
                print('(%s, %s, %0.3g)' % (str(n), str(nbr), wt))

    def reset(self):

        pass
