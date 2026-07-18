#!/usr/bin/env bash

set -euo pipefail

build_type="${BUILD_TYPE:-Release}"
build_dir="${BUILD_DIR:-build}"
install_prefix="${INSTALL_PREFIX:-$HOME/.local}"
generator="${CMAKE_GENERATOR:-Ninja}"

cmake \
    -S . \
    -B "${build_dir}" \
    -G "${generator}" \
    -DCMAKE_BUILD_TYPE="${build_type}" \
    -DCMAKE_PREFIX_PATH="${install_prefix}" \
    -DCMAKE_EXPORT_COMPILE_COMMANDS=ON \
    -DVORTEX_ENABLE_TRACY=ON

cmake \
    --build "${build_dir}" \
    --parallel
