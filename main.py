# local packages
import os_calls
from simulation import Simulation


def main():

    shouldSavePlots = True
    basePlotDir = 'figures'

    os_calls.clear_screen()

    sim = Simulation(shouldSavePlots, basePlotDir)

    # sim.run(simType='cspace')
    # sim.run(simType='gradient')
    sim.run(simType='wavefront')


if __name__ == '__main__':
    main()
