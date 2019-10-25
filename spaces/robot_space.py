# Generic class to implement a spatial representation of a robot or its
# environment for robots with 2-D workspaces
#
class RobotSpace:

    #
    # @brief      RobotSpace class constructor
    #
    # @param      shouldSavePlots  Boolean controlling whether or not the plt
    #                              objs can be saved to the baseSaveName dir
    # @param      baseSaveFName    The base directory file name for output plot
    #
    # @return     initialized RobotSpace object
    #
    def __init__(self, shouldSavePlots, baseSaveFName):

        self.obstacles = None

        self.shouldSavePlots = shouldSavePlots
        self.baseSaveFName = baseSaveFName


    #
    # @brief      An abstract function to compute obstacles in the RobotSpace
    #
    # @return     a list of obstacle coordinates for each obstacle in the
    #             space's coordinate system
    #
    def getObstacles(self):

        return NotImplementedError


    #
    # @brief      Abstract function ensuring that the subclass implements
    #             its own plotter 
    #       
    def plot(self):

        raise NotImplementedError
