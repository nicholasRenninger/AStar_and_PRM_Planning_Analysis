# local packages
from factory.object_factory import ObjectFactory
from spaces.workspace import WorkspaceBuilder

# registering the builders for the different types of space objects with a more
# readable interface to our generic factory class
#
class RobotSpaceCollection(ObjectFactory):

    #
    # @brief      allows for more readble creation / access to a concrete
    #             RobotSpace objects
    #
    # @param      robotSpaceType  The RobotSpace type @string
    # @param      kwargs          The keywords arguments to pass to the
    #                             specific robot constructor
    #
    # @return     the initialized instance to the robotSpaceType class
    #
    def get(self, robotSpaceType, **kwargs):

        return self.create(robotSpaceType, **kwargs)

activeSpaces = RobotSpaceCollection()
activeSpaces.register_builder('WORKSPACE', WorkspaceBuilder())
# activeSpaces.register_builder('CSPACE')