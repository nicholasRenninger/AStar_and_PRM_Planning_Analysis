import numpy as np
import math


class Link:

    def __init__(self, origin, linkLength, jointOffset):

        self.linkLength = linkLength
        self.jointOffset = jointOffset

        self.width = 1
        self.bl = None
        self.angle = None

        self.setOrigin(origin)

    def setOrigin(self, origin):
        self.origin = origin

    def setAngle(self, angle):
        self.angle = angle - math.pi / 2

    def setEdgeVertices(self, TransfMat):

        k = self.jointOffset
        w = self.width

        blLocal = np.array([[-k, w / 2, 1]]).T

        self.bl = np.matmul(TransfMat, blLocal)
