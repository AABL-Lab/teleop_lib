
_registry = dict()

try:
    from .gen2_robot import Gen2Plugin
    _registry["gen2"] = gen2_robot.Gen2Plugin
except ImportError:
    pass

try:
    from .publisher import PublisherPlugin
    _registry["publisher"] = PublisherPlugin
except ImportError:
    pass

def register_plugin(name, cls):
    _registry[name] = cls
def list_plugins():
    return list(_registry.keys())
def get_plugin(key):
    return _registry[key]
