import yaml
import numpy as np
import math
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle
from . import Link


class Manipulator:

    #
    # @brief      Manipulator class constructor
    #
    # @param      self             The Manipulator object object
    # @param      shouldSavePlots  Boolean controlling whether or not the plt
    #                              objs can be saved to the baseSaveName dir
    # @param      baseSaveFName    The base directory file name for output plot
    #
    # @return     initialized Manipulator object
    #
    def __init__(self, desEndEffectorLocation, desJointAngles, links,
                 shouldSavePlots, baseSaveFName):

        self.links = links
        self.numLinks = len(links)

        # start manipulator with all joints fully extended
        self.jointAngles = [0, 0, 0]

        if desJointAngles is not None:
            self.desJointAngles = desJointAngles
        else:
            self.desJointAngles = self.jointAngles

        print(self.desJointAngles)
        print(self.jointAngles)
        self.setEffectorToDesired()
        print(self.endEffectorLocation)

        # need to convert effector location to numpy arrays for easy
        # transformation
        if desEndEffectorLocation is not None:
            self.desEndEffectorLocation = np.array(desEndEffectorLocation)
        else:
            self.desEndEffectorLocation = self.endEffectorLocation

        self.shouldSavePlots = shouldSavePlots
        self.baseSaveFName = baseSaveFName

    #
    # @brief      reads in Link and kinematics parameters from YAML config file
    #
    # @param      configFileName  The YAML configuration file name
    #
    # @return     a list of workspace obstacle vertex coordinates for each
    #             obstacle
    #
    @staticmethod
    def getEnvFromFile(configFileName):

        with open(configFileName, 'r') as stream:
            config_data = yaml.load(stream, Loader=yaml.Loader)

        linkLengths = config_data['linkLengths']
        jointOffset = config_data['jointOffset']
        desJointAnglesSet = config_data['desJointAnglesSet']
        desEndEffectorLocations = config_data['desEndEffectorLocations']

        return (linkLengths, jointOffset,
                desJointAnglesSet, desEndEffectorLocations)

    def setEffectorToDesired(self):

        desJA = self.desJointAngles
        (self.links,
         self.endEffectorLocation) = self.forwardKinematics(self.links, desJA)

    def forwardKinematics(self, links, desJointAngles):

        i = 0
        a = np.zeros(self.numLinks + 1)
        i += 1
        T = np.zeros((3, 3, len(links)))
        A = np.array([[0, 0, 1]]).T

        # compute link origins in global coordinates
        angleSum = 0
        for (link, jointAngle) in zip(links, desJointAngles):
            angleSum += jointAngle
            c_i = math.cos(jointAngle)
            s_i = math.sin(jointAngle)
            a[i] = link.linkLength - 2 * link.jointOffset

            T[:, :, i - 1] = np.array([[c_i, -s_i, a[i - 1]],
                                       [s_i, c_i, 0],
                                       [0, 0, 1]])
            if i == 1:
                prevProd = T[:, :, i - 1]
            else:
                prevProd = np.matmul(prevProd, T[:, :, i - 1])

            linkOrigin = np.matmul(prevProd, A)
            link.setEdgeVertices(prevProd)
            link.setAngle(angleSum)
            link.setOrigin(linkOrigin)

            i += 1

        # now need to compute the end effector location
        T_i = np.array([[1, 0, a[i - 1]],
                        [0, 1, 0],
                        [0, 0, 1]])
        T = np.matmul(prevProd, T_i)

        endEffectorLocationXYZ = np.matmul(T, A)
        endEffectorLocation = np.array([endEffectorLocationXYZ[0],
                                        endEffectorLocationXYZ[1]])

        return (links, endEffectorLocation)

    #
    # @brief      Plot all workspace objects and saves to self.baseSaveFName
    #
    #             obstacles, the robot's path, the start location, and the goal
    #             location
    #
    # @param      self        The Workspace object
    #
    # @return     a plot of the manipulator in the self.baseSaveFName directory
    #
    def plot(self, plotTitle):

        fig = plt.figure()
        ax = fig.add_subplot(111)

        # plotting all the Links
        for link in self.links:
            bottomLeftVertex = link.bl

            width = link.width
            height = link.linkLength
            xy = (bottomLeftVertex[0], bottomLeftVertex[1])
            angle = link.angle * (180 / math.pi)

            p = Rectangle(xy, width, height, angle,
                          facecolor='black', edgecolor='red')
            ax.add_patch(p)

        # plotting the end effector
        endEffX = self.endEffectorLocation[0]
        endEffY = self.endEffectorLocation[1]
        plt.plot(endEffX, endEffY, color='blue', marker='o',
                 linestyle='dashed', linewidth=4, markersize=16)

        ax.set_aspect('equal')

        if self.shouldSavePlots:
            saveFName = self.baseSaveFName + '-' + plotTitle + '.png'
            fig = plt.gcf()
            fig.canvas.manager.full_screen_toggle()
            fig.show()
            fig.set_size_inches((8, 9), forward=False)
            plt.savefig(saveFName, dpi=300)
            print('wrote figure to ', saveFName)
        print('done')


#
# @brief      loads manipulator from a config file and
#             runs it with the desired kinematic commands
#
# @param      configFileName  The configuration file name
#
# @return     performs kinematic / inverse kinematic commands,
#             plots results
#
def runManipulator(configFileName, shouldSavePlots, baseSaveFName):

    (linkLengths,
     jointOffset,
     desJointAnglesSet,
     desEndEffectorLocations) = Manipulator.getEnvFromFile(configFileName)

    links = []
    for linkLength in linkLengths:
        link = Link.Link(origin=[0, 0], linkLength=linkLength,
                         jointOffset=jointOffset)
        links.append(link)

    # doing forward kinematics for each set of desired joint angles
    for desJointAngles in desJointAnglesSet:

        a1 = desJointAngles[0]
        a2 = desJointAngles[1]
        a3 = desJointAngles[2]
        plotTitle = 'forward_kin_%0.3g_%0.3g_%0.3g' % (a1, a2, a3)
        print(plotTitle)
        manipulator = Manipulator(desEndEffectorLocation=None,
                                  desJointAngles=desJointAngles,
                                  links=links,
                                  shouldSavePlots=shouldSavePlots,
                                  baseSaveFName=baseSaveFName)
        manipulator.plot(plotTitle)

    # doing inverse kinematics for each desired end effector location
    for desEndEffLoc in desEndEffectorLocations:

        x = desEndEffLoc[0]
        y = desEndEffLoc[1]
        plotTitle = 'inverse_kin_%0.3g_%0.3g' % (x, y)

        manipulator = Manipulator(desEndEffectorLocation=desEndEffLoc,
                                  desJointAngles=None,
                                  links=links,
                                  shouldSavePlots=shouldSavePlots,
                                  baseSaveFName=baseSaveFName)
        manipulator.plot(plotTitle)
