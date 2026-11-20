# Third-Party Software Notices

The EngineAI BSD 3-Clause license in `LICENSE.txt` applies to EngineAI-owned
content. The third-party components below remain under their respective
licenses. This file and the license texts under `licenses/` are part of every
source and binary distribution of the EngineAI RobotSession SDK.

The `.mnn` policy files and EngineAI RobotSession binaries are EngineAI-owned
artifacts and are not third-party software notices.

## Eclipse Cyclone DDS

- Component: Eclipse Cyclone DDS 0.10.5
- Upstream: https://github.com/eclipse-cyclonedds/cyclonedds/tree/0.10.5
- Distributed files: `lib/*/libddsc.so.0.10.5` and
  `lib/*/libddsc.so.0`
- Upstream license choice: EPL-2.0 OR EDL-1.0
- EngineAI distribution choice: Eclipse Distribution License 1.0
- License: `licenses/CycloneDDS-LICENSE.txt`
- Notice: `licenses/CycloneDDS-NOTICE.md`

EngineAI changes only the distributed ELF dynamic metadata so that RUNPATH is
`$ORIGIN`; no source-level Cyclone DDS functionality is modified.

Cyclone DDS includes the `libmd5-rfc` implementation by Aladdin Enterprises
under the Zlib license. Its license and copyright notice are reproduced in
`licenses/libmd5-rfc-Zlib.txt`. The `getopt` item identified by the upstream
NOTICE is public-domain content.

## Eclipse iceoryx

- Components: iceoryx binding_c, posh, hoofs, and platform
- Versions: 2.0.5 on Ubuntu 22.04; 2.0.6 on Ubuntu 24.04
- Upstream: https://github.com/eclipse-iceoryx/iceoryx
- Distributed files: `lib/*/libiceoryx_*.so`
- License: Apache License 2.0
- License text: `licenses/Apache-2.0.txt`
- Upstream notice: `licenses/iceoryx-NOTICE.md`

EngineAI changes the ELF dynamic metadata of `libiceoryx_binding_c.so`,
`libiceoryx_posh.so`, and `libiceoryx_hoofs.so` so that RUNPATH is `$ORIGIN`;
no source-level iceoryx functionality is modified.

The ROS iceoryx binary packages also carry cpptoml content:

- Component: cpptoml
- Copyright: Copyright (c) 2014 Chase Geigle
- License: MIT
- License text: `licenses/cpptoml-MIT.txt`

## MNN

- Component: MNN 2.9.5
- Upstream: https://github.com/alibaba/MNN
- Distributed files: `cpp/examples/third_party/mnn/include/**` and
  `cpp/examples/third_party/mnn/lib/*/libMNN.so`
- License: Apache License 2.0
- License text: `licenses/Apache-2.0.txt`

`MNN/Matrix.h` and `MNN/Rect.h` retain code and notices derived from Skia:

- Copyright: Copyright 2006 The Android Open Source Project
- License: BSD 3-Clause
- License text: `licenses/Skia-BSD-3-Clause.txt`

The distributed `libMNN.so` files also statically contain these MNN-bundled
components, confirmed on all three supported platforms:

- FlatBuffers: Apache License 2.0; covered by
  `licenses/Apache-2.0.txt`.
- half 1.12.0 / `half_float`: Copyright (c) 2012-2017 Christian Rau;
  MIT license in `licenses/half-MIT.txt`.

## nanobind and robin-map

nanobind is statically compiled into each distributed
`lib/*/_engineai_sdkpy*.so` Python extension.

- Component: nanobind 2.15.0
- Upstream: https://github.com/wjakob/nanobind/tree/v2.15.0
- License: BSD 3-Clause
- License text: `licenses/nanobind-BSD-3-Clause.txt`

nanobind's vendored robin-map headers are also compiled into the extension:

- Component: robin-map
- Upstream: https://github.com/Tessil/robin-map
- Copyright: Copyright (c) 2017 Thibaut Goetghebuer-Planchon
- License: MIT
- License text: `licenses/robin-map-MIT.txt`

## ROS 2 interface definitions and generation inputs

The RobotSession library contains generated C representations derived from the
following ROS 2 interface definitions and uses `rosidl_adapter` during the
build:

- `builtin_interfaces`
- `std_msgs`
- `geometry_msgs`
- `rosidl_adapter`

These ROS 2 packages are licensed under the Apache License 2.0. The license
text is reproduced in `licenses/Apache-2.0.txt`.

## System dynamic dependencies

The SDK binaries dynamically depend on system libraries including glibc,
libstdc++, libgcc, OpenSSL, and libacl. Copies of those system libraries are
not included in this repository. A container, Debian package, firmware image,
or other distribution that bundles them must perform its own artifact-level
license review.
