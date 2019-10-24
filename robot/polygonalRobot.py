from robot.robot import Robot


# @brief      This class describes a robot with a workspace shape that can be
#             described as a set of polygonal vertices 
class PolygonalRobot(Robot):

	#
    # @brief      PolygonalRobot class constructor
    #
    # @param      self             The PolygonalRobot object object
    # @param      configData       Configuration dictionary for the robot
    # @param      workspace        The Workspace object the robot operates in
    # @param      shouldSavePlots  Boolean controlling whether or not the plt
    #                              objs can be saved to the baseSaveName dir
    # @param      baseSaveFName    The base directory file name for output plot
    #
    # @return     initialized PolygonalRobot object
    #
    def __init__(self, configData, workspace,
                 shouldSavePlots, baseSaveFName):

        # calling the superclass contructor to inherit its properties
        Robot.__init__(self, configData, workspace, shouldSavePlots,
                       baseSaveFName)

        # this is a list of vertices defining the polygonal robot in workspace
        self.workspaceShapeVerts = self.setRobotShape(configData)


    #
    # @brief      Extracts and formats the robot shape data from the configData
    #
    # @param      configData  Configuration dictionary for the robot
    #
    def setRobotShape(self, configData):

        relativeShapeVerts = configData['robotShape']
        robotOriginIdx = configData['robotOriginVertIdx']
        robotOrigin = configData['robotOriginLocInWokspace']

        # as the shape of the robot in workspace is given as relative
        # coordinates from the robot workspace origin, we need to offset the
        # relative vertex coordinates by the origin to get the actual position
        # of the robot's verts in the workspace
        workspaceShapeVerts = [list(map(lambda rel, orig: rel + orig,
                                    relVert, robotOrigin))
                               for relVert in relativeShapeVerts]

        return (workspaceShapeVerts, robotOriginIdx)


