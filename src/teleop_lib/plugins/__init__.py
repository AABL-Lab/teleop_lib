
_registry = dict()

from .gen2_robot import Gen2Plugin
_registry["gen2"] = gen2_robot.Gen2Plugin

from .publisher import PublisherPlugin
_registry["publisher"] = PublisherPlugin

def list_plugins():
    return _registry.keys()
def get_plugin(key):
    return _registry[key]
