#!/usr/bin/env bash
set -euo pipefail

echo "Installing CasADi from pinned commit..."

# ---- Config ----
CASADI_COMMIT="f959d3175a444d763e4eda4aece48f4c5f4a6f90" #Stable version 3.7.2
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
