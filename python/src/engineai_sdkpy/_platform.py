"""Platform selection. Same ids as the C++ consumer Platform.cmake."""

import os
import platform
from pathlib import Path

# Must match include/engineai_sdk/version.h ENGINEAI_SDK_VERSION_STRING.
SDK_VERSION = "1.0.0"

VALID_PLATFORMS = (
    "ubuntu22.04-amd64",
    "ubuntu22.04-arm64",
    "ubuntu24.04-arm64",
)

SOABI = {
    "ubuntu22.04-amd64": "cpython-310-x86_64-linux-gnu",
    "ubuntu22.04-arm64": "cpython-310-aarch64-linux-gnu",
    "ubuntu24.04-arm64": "cpython-312-aarch64-linux-gnu",
}


def repo_root() -> Path:
    # python/src/engineai_sdkpy/_platform.py -> consumer repository root.
    return Path(__file__).resolve().parents[3]


def resolve_platform() -> str:
    chosen = os.environ.get("ENGINEAI_ROBOT_SESSION_PLATFORM", "").strip()
    if not chosen:
        chosen = _detect_platform()
    if chosen not in VALID_PLATFORMS:
        known = ", ".join(VALID_PLATFORMS)
        raise ImportError(
            f"unsupported RobotSession platform {chosen!r}. "
            f"Set ENGINEAI_ROBOT_SESSION_PLATFORM to one of: {known}"
        )
    return chosen


def extension_path(platform_id: str | None = None) -> Path:
    platform_id = platform_id or resolve_platform()
    filename = f"_engineai_sdkpy.{SOABI[platform_id]}.so"
    return repo_root() / "lib" / platform_id / filename


def _detect_platform() -> str:
    machine = platform.machine().lower()
    if machine in ("x86_64", "amd64"):
        cpu = "amd64"
    elif machine in ("aarch64", "arm64"):
        cpu = "arm64"
    else:
        raise ImportError(f"unsupported CPU {platform.machine()!r}")

    os_release = Path("/etc/os-release")
    if not os_release.is_file():
        raise ImportError("cannot read /etc/os-release to choose a RobotSession platform")
    version_id = ""
    for line in os_release.read_text(encoding="utf-8").splitlines():
        if line.startswith("VERSION_ID="):
            version_id = line.split("=", 1)[1].strip().strip('"')
            break
    if not version_id:
        raise ImportError("VERSION_ID is missing from /etc/os-release")
    return f"ubuntu{version_id}-{cpu}"
