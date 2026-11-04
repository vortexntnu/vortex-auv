#!/usr/bin/env bash
# Build the pinned dependency inside this package, without system installation.
set -euo pipefail
package_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
dependency_dir="$package_dir/.deps"
expected_commit=71a25ca36c084cbad1f872e812d6d97fbadfdb05
mkdir -p "$dependency_dir"
if [[ ! -d "$dependency_dir/gtsam/.git" ]]; then
  git clone --depth 1 --branch 4.3.0 https://github.com/borglab/gtsam.git "$dependency_dir/gtsam"
fi
if [[ "$(git -C "$dependency_dir/gtsam" rev-parse HEAD)" != "$expected_commit" ]]; then
  echo "GTSAM checkout is not the pinned 4.3.0 commit; refusing to overwrite it." >&2
  exit 1
fi
cmake -S "$dependency_dir/gtsam" -B "$dependency_dir/build" \
  -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX="$dependency_dir/install" \
  '-DCMAKE_INSTALL_RPATH=$ORIGIN' \
  -DGTSAM_BUILD_TESTS=OFF -DGTSAM_BUILD_EXAMPLES_ALWAYS=OFF \
  -DGTSAM_BUILD_TIMING_ALWAYS=OFF -DGTSAM_BUILD_UNSTABLE=OFF \
  -DGTSAM_BUILD_PYTHON=OFF -DGTSAM_WITH_TBB=OFF -DGTSAM_USE_SYSTEM_EIGEN=ON \
  -DGTSAM_TANGENT_PREINTEGRATION=ON -DGTSAM_LIEGROUP_PREINTEGRATION=OFF \
  -DGTSAM_BUILD_WITH_MARCH_NATIVE=OFF
cmake --build "$dependency_dir/build" --parallel "${GTSAM_BUILD_JOBS:-4}"
cmake --install "$dependency_dir/build"
