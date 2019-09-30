import OS_Calls
import Workspace
import BugRobot
import os.path


def main():

    shouldSavePlots = True
    basePlotDir = 'figures'

    OS_Calls.clear_screen()

    bugAlgorithmsRunner(shouldSavePlots, basePlotDir)


def bugAlgorithmsRunner(shouldSavePlots, basePlotDir):

    configDir = 'config'
    fType = '.yaml'
    configNames = ['scenario1', 'scenario2']
    configFileNames = [os.path.join(configDir, fName) + fType
                       for fName in configNames]

    for (file, configName) in zip(configFileNames, configNames):
        print('===============================')
        print(configName)
        print('===============================')
        baseSaveFName = os.path.join(basePlotDir, configName)
        currWorkspace = Workspace.Workspace(configFileName=file,
                                            shouldSavePlots=shouldSavePlots,
                                            baseSaveFName=baseSaveFName)

        algStr = 'bug1'
        BugRobot.runRobot(configFileName=file, currWorkspace=currWorkspace,
                          algorithmStr=algStr)

        algStr = 'bug2'
        BugRobot.runRobot(configFileName=file, currWorkspace=currWorkspace,
                          algorithmStr=algStr)


if __name__ == '__main__':
    main()
