# local packages
from factory.object_factory import ObjectFactory
from robots.polygonal_robot import PolygonalRobotBuilder
from robots.point_robot import PointRobotBuilder
from robots.manipulator_robot import ManipulatorRobotBuilder


# registering the builders for the different types of Robot objects with a more
# readable interface to our generic factory class
#
class RobotWarehouse(ObjectFactory):

    ##
    # @brief      allows for more readble creation / access to a concrete
    #             Robot object
    #
    # @param      robotType  The robot type string
    # @param      kwargs     The keywords arguments to pass to the specific
    #                        robot constructor
    #
    # @return     the initialized instance to the robotType class
    #
    def get(self, robotType, **kwargs):
        kwargs['robotType'] = robotType
        return self.create(robotType, **kwargs)


activeRobots = RobotWarehouse()
activeRobots.register_builder('POLYGONALROBOT', PolygonalRobotBuilder())
activeRobots.register_builder('POINTROBOT', PointRobotBuilder())
activeRobots.register_builder('MANIPULATOR', ManipulatorRobotBuilder())
