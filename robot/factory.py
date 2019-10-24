import robot.polygonalRobot as rp

#
# @brief      Front-end to create robot objects with different robotType s
#
class RobotFactory:

    #
    # @brief      Return a concrete Robot objects based on the robotType
    #
    # @param      robotType  The robotType string:
    #                                - 'polygonalRobot'
    #                                - 'manipulator'
    #                                - 'pointRobot'
    # @param      argv       The arguments array for the object type specified
    #                        by robotType
    # @param      self  The factory object
    #
    # @return     A concrete, specialized Robot object based on robotType
    #
    @staticmethod
    def getRobot(robotType, *argv):

        if robotType == 'polygonalRobot':

            return rp.PolygonalRobot(*argv)

        elif robotType == 'manipulator':

            return rp.Manipulator(argv)
        
        elif robotType == 'pointRobot':

            return rp.PointRobot(argv)

        else:
            raise ValueError(robotType)
