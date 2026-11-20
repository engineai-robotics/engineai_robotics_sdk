"""Load the prebuilt RobotSession extension for this platform.

Class and method names are listed in __init__.pyi.
"""

import importlib.util
import sys

from engineai_sdkpy._platform import SDK_VERSION, extension_path, resolve_platform

__all__ = [
    "BatteryInfo",
    "BaseVelocityPublisher",
    "BodyVelocityCommand",
    "ByteList",
    "DoubleList",
    "FloatList",
    "GamepadAxis",
    "GamepadButton",
    "GamepadKeys",
    "GamepadReader",
    "Heartbeat",
    "ImuInfo",
    "ImuReader",
    "IntList",
    "JointCommand",
    "JointCommandClient",
    "JointMotionPlanRequest",
    "JointMotionPlanRequestType",
    "JointMotionPlanState",
    "JointMotionPlanStatus",
    "JointMotionPlanner",
    "JointOverrideCommand",
    "JointOverridePublisher",
    "JointState",
    "JointStateReader",
    "LedColor",
    "LedController",
    "MotionController",
    "MotorCommandReader",
    "MotorDebug",
    "MotorDiagnosticsReader",
    "MotorStateReader",
    "NodeHeartbeatReader",
    "ParallelParserType",
    "PowerInfo",
    "PowerReader",
    "PowerSystemInfo",
    "Quaternion",
    "Robot",
    "SessionCreateResult",
    "SessionError",
    "SessionOptions",
    "StringList",
    "Vector3",
]


def _load_extension():
    platform_id = resolve_platform()
    path = extension_path(platform_id)
    if not path.is_file():
        raise ImportError(
            f"missing RobotSession extension for {platform_id}: {path}. "
            "This package does not compile the binding."
        )
    spec = importlib.util.spec_from_file_location("_engineai_sdkpy", path)
    if spec is None or spec.loader is None:
        raise ImportError(f"cannot load RobotSession extension: {path}")
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    version = getattr(module, "__version__", None)
    if version != SDK_VERSION:
        raise ImportError(
            f"{path} version {version!r} does not match version.h ({SDK_VERSION})"
        )
    return module


_ext = _load_extension()
__version__ = _ext.__version__

for _name in __all__:
    if not hasattr(_ext, _name):
        raise ImportError(f"RobotSession extension is missing {_name}")
    globals()[_name] = getattr(_ext, _name)
