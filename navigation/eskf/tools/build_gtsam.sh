#!/usr/bin/env bash
# Build the exact Forster implementation used by this backend.
set -euo pipefail
if [[ $# -ne 2 ]]; then
    echo "Usage: $0 BUILD_DIRECTORY INSTALL_PREFIX" >&2
    exit 2
fi
build_directory=$(realpath -m "$1")
install_prefix=$(realpath -m "$2")
revision=4f66a491ffc83cf092d0d818b11dc35135521612
mkdir -p "$build_directory"
if [[ ! -d "$build_directory/source" ]]; then
    git clone --depth 1 --branch 4.2 https://github.com/borglab/gtsam.git "$build_directory/source"
fi
if [[ $(git -C "$build_directory/source" rev-parse HEAD) != "$revision" ]]; then
    echo "GTSAM checkout does not match pinned 4.2 revision $revision" >&2
    exit 1
fi
if [[ -n $(git -C "$build_directory/source" status --porcelain) ]]; then
    echo "GTSAM source contains local changes; use a clean build directory" >&2
    exit 1
fi
cmake -S "$build_directory/source" -B "$build_directory/build" \
    -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX="$install_prefix" \
    -DGTSAM_TANGENT_PREINTEGRATION=OFF -DGTSAM_BUILD_UNSTABLE=ON \
    -DGTSAM_BUILD_TESTS=OFF -DGTSAM_BUILD_EXAMPLES_ALWAYS=OFF \
    -DGTSAM_BUILD_UNSTABLE_EXAMPLES_ALWAYS=OFF -DGTSAM_WITH_TBB=OFF \
    -DGTSAM_BUILD_WITH_MARCH_NATIVE=OFF -DGTSAM_USE_SYSTEM_EIGEN=ON
cmake --build "$build_directory/build" --parallel "${BUILD_JOBS:-2}"
cmake --install "$build_directory/build"
