import numpy as np
import math


class Link:

    def __init__(self, origin, linkLength, jointOffset):

        self.linkLength = linkLength
        self.jointOffset = jointOffset

        self.width = 2

        self.bl = None
        self.br = None
        self.tl = None
        self.tr = None

        self.setOrigin(origin)

        self.angle = None

    def setOrigin(self, origin):

        self.origin = origin

    def setAngle(self, angle):
        self.angle = angle - math.pi / 2

    def setEdgeVertices(self, TransfMat):

        ox = self.origin[0]
        oy = self.origin[1]
        length = self.linkLength
        k = self.jointOffset
        w = self.width

        blLocal = np.array([[-k, w / 2, 1]]).T
        tlLocal = np.array([[length + k, w / 2, 1]]).T

        self.bl = np.matmul(TransfMat, blLocal)
        self.tl = np.matmul(TransfMat, tlLocal)
