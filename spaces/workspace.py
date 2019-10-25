# 3rd-party packages
from spaces.robot_space import RobotSpace
from factory.builder import Builder


# @brief      Class for 2D robot workspace objects
#
# Workspace can only be comprised of rectangular obstacles
class Workspace(RobotSpace):

    #
    # @brief      Workspace class constructor
    #
    # @param      self             The workspace object object
    # @param      configData       Configuration dictionary for the robot
    #                              containing the obstacle coords
    # @param      shouldSavePlots  Boolean controlling whether or not the plt
    #                              objs can be saved to the baseSaveName dir
    # @param      baseSaveFName    The base directory file name for output plot
    #
    # @return     initialized workspace object
    #
    def __init__(self, configData, shouldSavePlots, baseSaveFName):

        obstacles = self.getObstacles(configData)
        self.obstacles = obstacles

        self.shouldSavePlots = shouldSavePlots
        self.baseSaveFName = baseSaveFName

    #
    # @brief      Creates a list of obstacles from config data
    #
    # @param      configData  configuration data dictionary for the robot
    #
    # @return     a list of workspace obstacle vertex coordinates for each
    #             obstacle
    #
    def getObstacles(self, configData):

        obstacles = configData['WO']
        return obstacles


#
# @brief      Implements the generic builder class for the Workspace
#
class WorkspaceBuilder(Builder):

    # need to call the super class constructor to gain its properties
    #
    def __init__(self):
        Builder.__init__(self)

    #
    # @brief      Implements the smart constructor for Workspace
    # 
    # Only reads the config data once, otherwise just returns the built object
    # 
    # @param      configFileName   The YAML configuration file name
    # @param      shouldSavePlots  The should save plots
    # @param      baseSaveFName    The base directory file name for output plot
    #
    def __call__(self, configFileName, shouldSavePlots, baseSaveFName):

        configNameHasChanged = (self._configName != configFileName)
        noInstanceLoadedYet = (self._instance is None)

        if noInstanceLoadedYet or configNameHasChanged:

            self._configName = configFileName
            configData = self.loadConfigData(configFileName)
            self._instance = Workspace(configData=configData,
                                       shouldSavePlots=shouldSavePlots,
                                       baseSaveFName=baseSaveFName)

        return self._instance
