# Changelog

The version matches `include/engineai_sdk/version.h`. Release tags on this repository use `vX.Y.Z`.

## [1.0.0]

### Added

- Public headers `include/engineai_sdk/robot.h`, `types.h`, and `version.h`, and the shared library `libEngineAIRobotSession.so.1`.
- Platforms `ubuntu22.04-amd64`, `ubuntu22.04-arm64`, and `ubuntu24.04-arm64`.
- C++ basic examples, scenario examples, and configs for pm01, t800, and t800pro, plus the MNN runtime, `build.sh`, and `clear.sh`, under `cpp/`.
- Shared example YAML in `cpp/examples/config/`. Python examples read that directory.
- Python binding `engineai_sdkpy`, with prebuilt extensions for the three platforms in `lib/<platform>/`.
- Python basic examples and scenario examples. `joint_bridge_stand_and_walk` is C++ only.
