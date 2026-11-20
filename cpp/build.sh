#!/usr/bin/env bash
# Customer-facing: build C++ examples against the repo-root SDK prefix.
# Run from cpp/: ./build.sh
set -euo pipefail

CPP_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${CPP_ROOT}/.." && pwd)"

if [[ ! -f "${CPP_ROOT}/examples/CMakeLists.txt" ]]; then
  echo "examples/CMakeLists.txt missing under ${CPP_ROOT}" >&2
  exit 1
fi

BUILD_DIR="${BUILD_DIR:-${CPP_ROOT}/build}"
export CMAKE_PREFIX_PATH="${REPO_ROOT}${CMAKE_PREFIX_PATH:+:${CMAKE_PREFIX_PATH}}"
cmake -S "${CPP_ROOT}/examples" -B "${BUILD_DIR}" -DCMAKE_PREFIX_PATH="${REPO_ROOT}"
cmake --build "${BUILD_DIR}" "$@"
echo "Examples built under ${BUILD_DIR}"
