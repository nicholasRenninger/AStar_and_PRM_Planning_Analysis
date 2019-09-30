import OS_Calls
import Workspace
import BugRobot
import os.path


def main():

    shouldSavePlots = True
    basePlotDir = 'figures'

    OS_Calls.clear_screen()

    bugAlgorithmRunner(shouldSavePlots, basePlotDir)


def bugAlgorithmRunner(shouldSavePlots, basePlotDir):

    configDir = 'config'
    fType = '.yaml'
    configNames = ['scenario1', 'scenario2']
    configFileNames = [os.path.join(configDir, fName) + fType
                       for fName in configNames]

    for (file, configName) in zip(configFileNames, configNames):

        baseSaveFName = os.path.join(basePlotDir, configName)
        currWorkspace = Workspace.Workspace(configFileName=file,
                                            shouldSavePlots=shouldSavePlots,
                                            baseSaveFName=baseSaveFName)

        algStr = 'bug1'
        bug1Robot = BugRobot.BugRobot(configFileName=file,
                                      workspace=currWorkspace,
                                      algorithmStr=algStr)
        bug1Robot.deploy()
        currWorkspace.plot(robotPath=bug1Robot.stateHistory,
                           startState=bug1Robot.startState,
                           goalState=bug1Robot.goalState,
                           plotTitle=algStr)


if __name__ == '__main__':
    main()
