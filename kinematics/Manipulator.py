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

        self.shouldSavePlots = shouldSavePlots
        self.baseSaveFName = baseSaveFName

        self.links = links
        self.numLinks = len(links)
        self.IK_TOL = 1e-6

        # start manipulator with all joints fully extended
        self.jointAngles = [0, 0, 0]
        self.endEffectorLocation = [0, 0]

        if desJointAngles is not None:
            self.desJointAngles = desJointAngles
            self.setAnglesToDesired()
        else:
            self.desJointAngles = self.jointAngles

        # need to convert effector location to numpy arrays for easy
        # transformation
        if desEndEffectorLocation is not None:
            self.desEndEffectorLocation = np.array(desEndEffectorLocation)
            self.setEffectorToDesired()
        else:
            self.desEndEffectorLocation = self.endEffectorLocation

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

    def setAnglesToDesired(self):

        desJA = self.desJointAngles
        (self.links,
         self.endEffectorLocation) = self.forwardKinematics(self.links, desJA)

        print('joint angles: ', self.jointAngles)
        print('effector at: ', self.endEffectorLocation)

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

    def setEffectorToDesired(self):

        desEff = self.desEndEffectorLocation
        print('desired end effector pos:', desEff)
        (self.links,
         self.jointAngles) = self.inverseKinematics(self.links, desEff)

        print('joint angles: ', self.jointAngles)
        print('effector at: ', self.endEffectorLocation)

    def inverseKinematics(self, links, desEndEffectorLocation):

        x = desEndEffectorLocation[0]
        y = desEndEffectorLocation[1]

        a1 = links[0].linkLength - 2 * links[0].jointOffset
        a2 = links[1].linkLength - 2 * links[1].jointOffset
        a3 = links[2].linkLength - 2 * links[2].jointOffset

        possibleTheta2 = []
        possibleTheta3 = []

        possibleTheta1 = np.linspace(0, math.pi, num=4)

        for theta1 in possibleTheta1:

            # locking one link in place - theta1 in [0, pi]
            link1Origin = [0, 0]
            lx = link1Origin[0] + a1 * math.cos(theta1)
            ly = link1Origin[1] + a1 * math.sin(theta1)
            print(lx, ly)

            # adjust x and y so the system is solved assuming the link 2 and 3
            # start at the origin
            x -= lx
            y -= ly

            # trying the rest of the angles
            cosTheta3 = (1 / (2 * a2 * a3)) * ((x**2 + y**2) -
                                               (a2**2 + a3**2))
            sinTheta3_1 = math.sqrt(1 - cosTheta3**2)
            sinTheta3_2 = -math.sqrt(1 - cosTheta3**2)

            possibleTheta3.append(math.atan2(sinTheta3_1, cosTheta3))
            possibleTheta3.append(math.atan2(sinTheta3_2, cosTheta3))

            k = (a2 + a3 * cosTheta3)
            xi = a3 * math.sqrt(1 - cosTheta3**2)
            mexican = (1 / (x**2 + y**2))
            cosTheta2_1 = mexican * (x * k + y * xi)
            cosTheta2_2 = mexican * (x * k - y * xi)

            sinTheta2_1 = mexican * (y * k - x * xi)
            sinTheta2_2 = mexican * (y * k + x * xi)

            possibleTheta2.append(math.atan2(sinTheta2_1, cosTheta2_1))
            possibleTheta2.append(math.atan2(sinTheta2_1, cosTheta2_2))
            possibleTheta2.append(math.atan2(sinTheta2_2, cosTheta2_1))
            possibleTheta2.append(math.atan2(sinTheta2_2, cosTheta2_2))

            for theta2 in possibleTheta2:
                for theta3 in possibleTheta3:
                    desJA = [theta1, theta2, theta3]
                    print('trying: ', desJA)
                    (ll, endEffLoc) = self.forwardKinematics(links, desJA)
                    self.endEffectorLocation = endEffLoc
                    print('effector pos:', endEffLoc)
                    xSolve = self.endEffectorLocation[0]
                    ySolve = self.endEffectorLocation[1]

                    totalError = abs(x + lx - xSolve) + abs(y + ly - ySolve)
                    if totalError < self.IK_TOL:
                        return (links, desJA)

        return (links, desJA)

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
