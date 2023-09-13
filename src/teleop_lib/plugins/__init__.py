
import functools

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

try:
    from .gen3_robot import Gen3Plugin
except ImportError:
    raise
    pass
else:
    _registry['gen3'] = functools.partial(Gen3Plugin, "gen3")
    _registry["gen3_lite"] = functools.partial(Gen3Plugin, "gen3_lite")

def register_plugin(name, cls):
    _registry[name] = cls
def list_plugins():
    return list(_registry.keys())
def get_plugin(key):
    return _registry[key]
