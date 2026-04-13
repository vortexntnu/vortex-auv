#!/usr/bin/env bash
set -euo pipefail

echo "Installing Control Toolbox..."

# ---- Config ----
WORKSPACE_DIR="${HOME}/third_party/ct_ws"
SRC_DIR="${WORKSPACE_DIR}/src"
INSTALL_PREFIX="${HOME}/.local"
BUILD_TYPE="Release"

KINDR_REPO="https://github.com/ANYbotics/kindr.git"
CT_REPO="https://github.com/ethz-adrl/control-toolbox.git"

KINDR_DIR="${SRC_DIR}/kindr"
CT_DIR="${SRC_DIR}/control-toolbox"

mkdir -p "${SRC_DIR}"
mkdir -p "${INSTALL_PREFIX}"

echo "Workspace: ${WORKSPACE_DIR}"
echo "Install prefix: ${INSTALL_PREFIX}"

# ---- System deps ----
sudo apt-get update
sudo apt-get install -y \
    git \
    build-essential \
    cmake \
    libeigen3-dev \
    libboost-all-dev

# ---- Clone/update kindr ----
if [ ! -d "${KINDR_DIR}/.git" ]; then
    git clone "${KINDR_REPO}" "${KINDR_DIR}"
else
    git -C "${KINDR_DIR}" pull --ff-only
fi

# ---- Build/install kindr ----
cmake -S "${KINDR_DIR}" -B "${KINDR_DIR}/build" \
    -DUSE_CMAKE=true \
    -DCMAKE_BUILD_TYPE="${BUILD_TYPE}" \
    -DCMAKE_INSTALL_PREFIX="${INSTALL_PREFIX}"

cmake --build "${KINDR_DIR}/build" -j"$(nproc)"
cmake --install "${KINDR_DIR}/build"

# ---- Clone/update control-toolbox ----
if [ ! -d "${CT_DIR}/.git" ]; then
    git clone "${CT_REPO}" "${CT_DIR}"
else
    git -C "${CT_DIR}" pull --ff-only
fi

# ---- Build CT using the provided script ----
chmod +x "${CT_DIR}/ct/build_ct.sh"

export CMAKE_PREFIX_PATH="${INSTALL_PREFIX}:${CMAKE_PREFIX_PATH:-}"

pushd "${CT_DIR}/ct" >/dev/null
./build_ct.sh \
    -DCMAKE_BUILD_TYPE="${BUILD_TYPE}" \
    -DCMAKE_PREFIX_PATH="${INSTALL_PREFIX}"
popd >/dev/null

echo
echo "Control Toolbox build complete."
echo "You may want this in your shell before building your ROS 2 package:"
echo "  export CMAKE_PREFIX_PATH=\"${INSTALL_PREFIX}:\$CMAKE_PREFIX_PATH\""
echo
echo "CT source: ${CT_DIR}"
echo "Kindr install: ${INSTALL_PREFIX}"