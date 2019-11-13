# local packages
import util.os_calls as os_calls
from simulation import Simulation


def main():

    # clear the screen of the previous output
    os_calls.clear_screen()

    # create an instance of the simulation class
    sim = Simulation(shouldSavePlots=True, basePlotDir='figures')

    # simulation runner handles all class interfaces for each type of sim
    # sim.run(simType='polygonalRobot')
    # sim.run(simType='gradient')
    # sim.run(simType='wavefront')
    # sim.run(simType='manipulator')
    # sim.run(simType='graphSearch')
    # sim.run(simType='prmPointRobot')
    sim.run(simType='prmPointRobotBenchmark')


if __name__ == '__main__':
    main()
