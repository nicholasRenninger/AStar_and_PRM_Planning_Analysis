# 3rd-party packages
import numpy as np
import math
from matplotlib.patches import Rectangle


##
# @brief      This class describes a link of a manipulatorRobot
#
class Link:

    ##
    # @brief      Constructs a new instance of a manipulator Link
    #
    # @param      origin       The link's origin
    # @param      linkLength   The link's length
    # @param      jointOffset  The joint offset from the ends of the link
    #
    def __init__(self, origin, linkLength, jointOffset):

        self.linkLength = linkLength
        self.jointOffset = jointOffset

        self.width = 0.05
        self.bl = None
        self.br = None
        self.tl = None
        self.tr = None
        self.angle = None

        self.setOrigin(origin)

    ##
    # @brief      Sets the origin of the link
    #
    # @param      origin  The origin coordinates of the link
    #
    def setOrigin(self, origin):

        self.origin = origin

    ##
    # @brief      Sets the rotational angle of the link body in global coords
    #
    # @param      angle  The new link body frame angle in global coords
    #
    def setAngle(self, angle):

        # I don't remember why :(
        self.angle = angle - math.pi / 2

    ##
    # @brief      Rotates the whole link with a transformation matrix
    #
    # @param      TransfMat  The rotation / translation matrix to apply to the
    #                        link
    #
    # @return     transforms the link edge vertex global coordinates with
    #             transfMat
    #
    def setEdgeVertices(self, TransfMat):

        k = self.jointOffset
        w = self.width

        # calculating the vertex offsets from the link local body frame
        blLocal = np.array([[-k, w / 2, 1]]).T
        brLocal = np.array([[-k, -w / 2, 1]]).T
        tlLocal = np.array([[self.linkLength - k, w / 2, 1]]).T
        trLocal = np.array([[self.linkLength - k, -w / 2, 1]]).T

        # multiplying the local frame coordinates of each vertex by the
        # transformation matrix to get them all in the global coordinate
        # system
        self.bl = np.matmul(TransfMat, blLocal)
        self.br = np.matmul(TransfMat, brLocal)
        self.tl = np.matmul(TransfMat, tlLocal)
        self.tr = np.matmul(TransfMat, trLocal)

    ##
    # @brief      Gets a list of the global coordinates of the vertices of the
    #             link
    #
    # @return     a list of edge vertices in global coords
    #
    def getEdgeVertices(self):

        return [self.bl, self.tl, self.tr, self.br]

    ##
    # @brief      Plots the link on ax
    #
    # @param      ax    the matplotlib Axes on which to plot the link
    #
    def plot(self, ax=None):

        verts = self.getEdgeVertices()
        bl = verts[0]

        width = self.width
        height = self.linkLength
        angle = self.angle * (180 / math.pi)

        p = Rectangle(bl, width, height, angle,
                      facecolor='grey', edgecolor='red')

        ax.add_patch(p)
