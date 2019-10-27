# 3rd-party packages


# local packages
from planners.planner import Planner
from factory.builder import Builder


##
# @brief      This class describes a Planner subclass used to find a path
#             in two-dimensional configuration space using gradient descent
#             algorithms
#
class GradientPlanner(Planner):

    ##
    # @brief      GradientPlanner class constructor
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

    ##
    # @brief      Computes a viable path in robot's cSpace to the goalState
    #             from the robot's startState
    #
    # @return     a viable set of cSpace states from startState to goalState
    #
    def findPathToGoal(self):

        return NotImplementedError


##
# @brief      Implements the generic builder class for the GradientPlanner
#
class GradientPlannerBuilder(Builder):

    ##
    # @brief need to call the super class constructor to gain its properties
    #
    def __init__(self):
        Builder.__init__(self)
        self.robot = None
        self.workspace = None

    ##
    # @brief      Implements the smart constructor for GradientPlanner
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
    # @return     instance of an initialized GradientPlanner object
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
            self._instance = GradientPlanner(plannerType=plannerType,
                                             cSpace=cSpace,
                                             workspace=workspace,
                                             robot=robot,
                                             configData=configData,
                                             shouldSavePlots=shouldSavePlots,
                                             baseSaveFName=baseSaveFName)

        return self._instance
