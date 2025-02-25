#!/usr/bin/env bash
# requirements.sh
# Installs dependencies not handled by rosdep (primarily Python libs).

set -e  # Exit on any error

echo "Installing additional Python dependencies..."

# Upgrade pip first
python3 -m pip install --upgrade pip

# Example Python packages that rosdep doesn't handle:
python3 -m pip install control==0.10.1
python3 -m pip install "numpy<1.25.0"

echo "Done installing additional dependencies."
