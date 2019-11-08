# local packages
from factory.object_factory import ObjectFactory
from planners.gradient_planner import GradientPlannerBuilder
from planners.wavefront_planner import WavefrontPlannerBuilder


# registering the builders for the different types of Robot objects with a more
# readable interface to our generic factory class
#
class PlannerCreator(ObjectFactory):

    ##
    # @brief      allows for more readble creation / access to a concrete
    #             Planner object
    #
    # @param      plannerType  The Planner type string
    # @param      kwargs       The keywords arguments to pass to the specific
    #                          robot constructor
    #
    # @return     the initialized instance to a subclass of the Planner class
    #
    def get(self, plannerType, **kwargs):
        kwargs['plannerType'] = plannerType
        return self.create(plannerType, **kwargs)


availablePlanners = PlannerCreator()
availablePlanners.register_builder('GRADIENT', GradientPlannerBuilder())
availablePlanners.register_builder('WAVEFRONT', WavefrontPlannerBuilder())
availablePlanners.register_builder('PRM', WavefrontPlannerBuilder())
