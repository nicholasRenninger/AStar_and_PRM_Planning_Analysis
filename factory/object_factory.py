

##
# @brief      Generic object factory leveraging the generic Builder interface
#             see: https://realpython.com/factory-method-python/
#
class ObjectFactory:

    ##
    # @brief      Constructs a new instance of the ObjectFactory
    #
    def __init__(self):

        self._builders = {}

    ##
    # @brief      adds the builder object to the internal builder dictionary
    #
    # @param      key      The string key reffering to the builder
    # @param      builder  The builder object
    #                      This can be any function, class or object
    #                      implementing the .__call__() method
    #
    # @return     the _builders dictionary has the key-builder pair appended
    #
    def register_builder(self, key, builder):

        self._builders[key] = builder

    ##
    # @brief      Returns a concrete object using the object builder specified
    #             by key and the concrete constructor arguements needed as
    #             kwargs
    #
    # @param      key       The string key specifying the builder to use
    # @param      **kwargs  The keywords arguments needed by the builder
    #                       specified by key
    #
    # @return     A concrete object built by the builder specified with key
    #
    def create(self, key, **kwargs):

        builder = self._builders.get(key)

        if not builder:
            raise ValueError(key)

        return builder(**kwargs)
