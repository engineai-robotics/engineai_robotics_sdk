#!/usr/bin/env bash
# Remove generated files under cpp/build.
# Run from cpp/: ./clear.sh
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
BUILD_DIR="${BUILD_DIR:-${ROOT}/build}"

if [[ "${BUILD_DIR}" == "${ROOT}" || "${BUILD_DIR}" == "${ROOT}/" ]]; then
  echo "refusing to clean the repository root" >&2
  exit 1
fi
case "${BUILD_DIR}" in
  "${ROOT}/"*) ;;
  *)
    echo "refusing to clean ${BUILD_DIR}: it is outside ${ROOT}" >&2
    exit 1
    ;;
esac

if [[ ! -d "${BUILD_DIR}" ]]; then
  echo "No build directory at ${BUILD_DIR}"
  exit 0
fi

find "${BUILD_DIR}" -mindepth 1 -exec rm -rf {} +
echo "Cleared ${BUILD_DIR}"
