import os.path
import yaml
import OS_Calls
from spaces.workspace import Workspace
from robot.factory import RobotFactory


def main():

    shouldSavePlots = True
    basePlotDir = 'figures'

    OS_Calls.clear_screen()

    simType = 'cspace'
    simRunner(shouldSavePlots, basePlotDir, simType)


def simRunner(shouldSavePlots, basePlotDir, simType):

    (configNames, configFileNames) = getConfigPaths(simType)

    for (file, configName) in zip(configFileNames, configNames):
        print('===============================')
        print(configName)
        print('===============================')
        baseSaveFName = os.path.join(basePlotDir, configName)

        if simType == 'cspace':
            runCspaceViz(file, shouldSavePlots, baseSaveFName)


# @brief      A function to interface with the classes to visualize the c-space
#             obstacle for worspace polygonal robot and obstacle
#
# @param      configFileName   The configuration file name for the simulation
# @param      shouldSavePlots  Boolean to turn on and off plot file writes
# @param      baseSaveFName    The base save file name for plots
#
def runCspaceViz(configFileName, shouldSavePlots, baseSaveFName):
    
    # get the configuration data for the whole problem
    configData = loadConfigData(configFileName)

    # the workspace doesn't change for this simulation
    currWorkspace = Workspace(configData=configData,
                                               shouldSavePlots=shouldSavePlots,
                                               baseSaveFName=baseSaveFName)

    # this simulation is for a polygonal robot, so get that class from the
    # factory
    robotType = 'polygonalRobot'
    currRobot = RobotFactory.getRobot(robotType, configData, currWorkspace,
                                      shouldSavePlots, baseSaveFName)


#
# @brief      Gets the configuration file paths for the given sim type
#
# @param      simType  The simulation type @string
#
# @return     The configuration path strings in a @list
#
def getConfigPaths(simType):

    if simType == 'cspace':
        configNames = ['WO_Rob_triangles', 'WO_Rob_triangles_no_rot']

    fullConfigNames = list(map(lambda x: simType + '_' + x, configNames))

    configDir = 'config'
    fType = '.yaml'
    configFileNames = [os.path.join(configDir, fName) + fType
                       for fName in fullConfigNames]

    return (fullConfigNames, configFileNames)


#
# @brief      reads in the simulation parameters from a YAML config file
#
# @param      configFileName  The YAML configuration file name
#
# @return     configuration data dictionary for the simulation
#
def loadConfigData(configFileName):

    with open(configFileName, 'r') as stream:
        configData = yaml.load(stream, Loader=yaml.Loader)

    return configData


if __name__ == '__main__':
    main()
