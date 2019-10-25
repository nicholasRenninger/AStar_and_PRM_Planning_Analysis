# 3rd-party packages
import os.path
import matplotlib.pyplot as plt

# local packages
import os_calls
from spaces.factory import activeSpaces
from robots.factory import activeRobots


def main():

    shouldSavePlots = True
    basePlotDir = 'figures'

    os_calls.clear_screen()

    simType = 'cspace'
    simRunner(shouldSavePlots, basePlotDir, simType)


#
# @brief      Runs a set of simulations based on the simulation type and
#             outputs the plots to the
#
# @param      shouldSavePlots  Boolean to turn on and off plot file writes
# @param      basePlotDir      The relative path to the figures output dir
# @param      simType          The simulation type @string
#       
def simRunner(shouldSavePlots, basePlotDir, simType):

    (configNames, configFileNames) = getConfigPaths(simType)

    for (file, configName) in zip(configFileNames, configNames):
        print('===============================')
        print(configName)
        print('===============================')
        baseSaveFName = os.path.join(basePlotDir, configName)

        if simType == 'cspace':
            runCspaceViz(file, shouldSavePlots, baseSaveFName)

    plt.show()


# @brief      A function to interface with the classes to visualize the c-space
#             obstacle for worspace polygonal robot and obstacle
#
# @param      configFileName   The configuration file name for the simulation
# @param      shouldSavePlots  Boolean to turn on and off plot file writes
# @param      baseSaveFName    The base save file name for plots
#
def runCspaceViz(configFileName, shouldSavePlots, baseSaveFName):
    
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

    currRobot.runAndPlot(planner=None, plotTitle='cspace_no_rot')


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


if __name__ == '__main__':
    main()
