#!/usr/bin/env bash
set -euo pipefail

echo "Installing CasADi from pinned commit..."

# ---- Config ----
CASADI_COMMIT="9b9f2c92e8b8bbd3a6c1e5e91c0a2b4dc1f88a62"   # example – change to what you want
CASADI_DIR="/tmp/casadi-src"

# ---- System deps ----
sudo apt-get update -qq
sudo apt-get install -y --no-install-recommends \
  git \
  cmake \
  build-essential \
  pkg-config \
  libblas-dev \
  liblapack-dev \
  libopenblas-dev \
  libgfortran5 \
  libeigen3-dev \
  libboost-dev

# ---- Clone + checkout ----
rm -rf "${CASADI_DIR}"
git clone https://github.com/casadi/casadi.git "${CASADI_DIR}"
cd "${CASADI_DIR}"
git checkout "${CASADI_COMMIT}"

# ---- Configure ----
mkdir build
cd build

cmake .. \
  -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_INSTALL_PREFIX=/usr/local \
  -DWITH_IPOPT=OFF \
  -DWITH_QPOASES=OFF \
  -DWITH_OSQP=OFF \
  -DWITH_SUNDIALS=OFF \
  -DWITH_MATLAB=OFF \
  -DWITH_PYTHON=OFF

# ---- Build + install ----
make -j$(nproc)
sudo make install
sudo ldconfig
echo "CasADi installed from source."
