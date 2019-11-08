# 3rd-party packages
import os.path

# local packages
from spaces.factory import activeSpaces
from robots.factory import activeRobots
from planners.factory import availablePlanners


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

            if simType == 'cspace':
                self.runCspaceViz(file, self.shouldSavePlots, baseSaveFName)

            if simType == 'gradient':
                self.runGradientPlanner(file, self.shouldSavePlots,
                                        baseSaveFName)
            if simType == 'wavefront':
                self.runWavefrontPlanner(file, self.shouldSavePlots,
                                         baseSaveFName)
            if simType == 'manipulator':
                self.runManipulator(file, self.shouldSavePlots,
                                    baseSaveFName)
            if simType == 'graphSearch':
                self.runGraphSearch(file, self.shouldSavePlots,
                                    baseSaveFName)

    ##
    # @brief      A function to interface with the classes to visualize the
    #             c-space obstacle for worspace polygonal robot and obstacle
    #
    # @param      configFileName   The configuration file name for the
    #                              simulation
    # @param      shouldSavePlots  Boolean to turn on and off plot file writes
    # @param      baseSaveFName    The base save file name for plots
    #
    def runCspaceViz(self, configFileName, shouldSavePlots, baseSaveFName):

        # the workspace doesn't change for this simulation
        currWorkspace = activeSpaces.get(robotSpaceType='WORKSPACE',
                                         configFileName=configFileName,
                                         shouldSavePlots=shouldSavePlots,
                                         baseSaveFName=baseSaveFName)

        # this simulation is for a polygonal robot, so get that class from the
        # activeRobots factory interface
        currRobot = activeRobots.get(robotType='POLYGONALROBOT',
                                     configFileName=configFileName,
                                     workspace=currWorkspace,
                                     shouldSavePlots=shouldSavePlots,
                                     baseSaveFName=baseSaveFName)

        currRobot.runAndPlot(planner=None, plotTitle='')

    ##
    # @brief      A function to interface with the classes to run a gradient
    #             based planner on a variety of environments
    #
    # @param      configFileName   The configuration file name for the
    #                              simulation
    # @param      shouldSavePlots  Boolean to turn on and off plot file writes
    # @param      baseSaveFName    The base save file name for plots
    #
    def runGradientPlanner(self, configFileName, shouldSavePlots,
                           baseSaveFName):

        # the workspace doesn't change for this simulation
        currWorkspace = activeSpaces.get(robotSpaceType='WORKSPACE',
                                         configFileName=configFileName,
                                         shouldSavePlots=shouldSavePlots,
                                         baseSaveFName=baseSaveFName)

        currRobot = activeRobots.get(robotType='POINTROBOT',
                                     configFileName=configFileName,
                                     workspace=currWorkspace,
                                     shouldSavePlots=shouldSavePlots,
                                     baseSaveFName=baseSaveFName)

        currPlanner = availablePlanners.get(plannerType='GRADIENT',
                                            cSpace=currRobot.cSpace,
                                            workspace=currWorkspace,
                                            robot=currRobot,
                                            configFileName=configFileName,
                                            shouldSavePlots=shouldSavePlots,
                                            baseSaveFName=baseSaveFName)

        currRobot.runAndPlot(planner=currPlanner, plotTitle='')

    ##
    # @brief      A function to interface with the classes to run a wavefront
    #             planner on a variety of environments
    #
    # @param      configFileName   The configuration file name for the
    #                              simulation
    # @param      shouldSavePlots  Boolean to turn on and off plot file writes
    # @param      baseSaveFName    The base save file name for plots
    #
    def runWavefrontPlanner(self, configFileName, shouldSavePlots,
                            baseSaveFName):

        # the workspace doesn't change for this simulation
        currWorkspace = activeSpaces.get(robotSpaceType='WORKSPACE',
                                         configFileName=configFileName,
                                         shouldSavePlots=shouldSavePlots,
                                         baseSaveFName=baseSaveFName)

        currRobot = activeRobots.get(robotType='POINTROBOT',
                                     configFileName=configFileName,
                                     workspace=currWorkspace,
                                     shouldSavePlots=shouldSavePlots,
                                     baseSaveFName=baseSaveFName)

        currPlanner = availablePlanners.get(plannerType='WAVEFRONT',
                                            cSpace=currRobot.cSpace,
                                            workspace=currWorkspace,
                                            robot=currRobot,
                                            configFileName=configFileName,
                                            shouldSavePlots=shouldSavePlots,
                                            baseSaveFName=baseSaveFName)

        currRobot.runAndPlot(planner=currPlanner, plotTitle='')

    ##
    # @brief      A function to interface with the classes to run a manipulator
    #             robot on a variety of environments
    #
    # @param      configFileName   The configuration file name for the
    #                              simulation
    # @param      shouldSavePlots  Boolean to turn on and off plot file writes
    # @param      baseSaveFName    The base save file name for plots
    #
    def runManipulator(self, configFileName, shouldSavePlots, baseSaveFName):

        # the workspace doesn't change for this simulation
        currWorkspace = activeSpaces.get(robotSpaceType='WORKSPACE',
                                         configFileName=configFileName,
                                         shouldSavePlots=shouldSavePlots,
                                         baseSaveFName=baseSaveFName)

        # this simulation is for a manipulator robot, so get that class from
        # the activeRobots factory interface
        currRobot = activeRobots.get(robotType='MANIPULATOR',
                                     configFileName=configFileName,
                                     workspace=currWorkspace,
                                     shouldSavePlots=shouldSavePlots,
                                     baseSaveFName=baseSaveFName)

        currPlanner = availablePlanners.get(plannerType='WAVEFRONT',
                                            cSpace=currRobot.cSpace,
                                            workspace=currWorkspace,
                                            robot=currRobot,
                                            configFileName=configFileName,
                                            shouldSavePlots=shouldSavePlots,
                                            baseSaveFName=baseSaveFName)

        currRobot.runAndPlot(planner=currPlanner, plotTitle='')

    ##
    # @brief      A function to interface with the classes to run
    #
    # @param      configFileName   The configuration file name for the
    #                              simulation
    # @param      shouldSavePlots  Boolean to turn on and off plot file writes
    # @param      baseSaveFName    The base save file name for plots
    #
    def runGraphSearch(self, configFileName, shouldSavePlots, baseSaveFName):

        # the workspace doesn't change for this simulation
        currWorkspace = activeSpaces.get(robotSpaceType='WORKSPACE',
                                         configFileName=configFileName,
                                         shouldSavePlots=shouldSavePlots,
                                         baseSaveFName=baseSaveFName)

        # this simulation is for a manipulator robot, so get that class from
        # the activeRobots factory interface
        currRobot = activeRobots.get(robotType='MANIPULATOR',
                                     configFileName=configFileName,
                                     workspace=currWorkspace,
                                     shouldSavePlots=shouldSavePlots,
                                     baseSaveFName=baseSaveFName)

        currPlanner = availablePlanners.get(plannerType='WAVEFRONT',
                                            cSpace=currRobot.cSpace,
                                            workspace=currWorkspace,
                                            robot=currRobot,
                                            configFileName=configFileName,
                                            shouldSavePlots=shouldSavePlots,
                                            baseSaveFName=baseSaveFName)

        currRobot.runAndPlot(planner=currPlanner, plotTitle='')

    ##
    # @brief      Gets the configuration file paths for the  given sim type
    #
    # @param      simType  The simulation type string
    #
    # @return     The configuration path strings in a list
    #
    def getConfigPaths(self, simType):

        if simType == 'cspace':
            configNames = ['WO_Rob_triangles', 'WO_Rob_triangles_no_rot']

        elif simType == 'gradient':
            configNames = ['env1', 'env2', 'env3']

        elif simType == 'wavefront':
            configNames = ['env2', 'env3']

        elif simType == 'manipulator':
            configNames = ['env1', 'env2', 'env3']

        else:
            raise ValueError(simType)

        fullConfigNames = list(map(lambda x: simType + '_' + x, configNames))

        configDir = 'config'
        fType = '.yaml'
        configFileNames = [os.path.join(configDir, fName) + fType
                           for fName in fullConfigNames]

        return (fullConfigNames, configFileNames)
