#!/usr/bin/env bash
set -euo pipefail

# ------------------------------------------------------------------------------
# Set environment variables for the image name.
# Defaulted to latest image.
# ------------------------------------------------------------------------------
IMAGE="ghcr.io/vortexntnu/auv-image:latest"

# ------------------------------------------------------------------------------
# Detect the target platform again if needed (though for 'docker run' it often
# isn't used unless you do special checks).
# ------------------------------------------------------------------------------
if [[ "$(uname)" == "Darwin" ]]; then
    PLATFORM="arm64"
else
    PLATFORM="arm64"
fi

# ------------------------------------------------------------------------------
# Locate this script and the project root (mounted as a volume).
# ------------------------------------------------------------------------------
SCRIPT_DIR="$(dirname "$(realpath "$0")")"
WORKSPACE="$(realpath "$SCRIPT_DIR/../../..")"

echo "======================================================================"
echo " Running container from image '$IMAGE'"
echo "   * PLATFORM (host architecture): $PLATFORM"
echo "   * Volume mount:                 $WORKSPACE -> /ros_ws"
echo "======================================================================"
echo ""

# ------------------------------------------------------------------------------
# Run the Docker container
# ------------------------------------------------------------------------------
docker run -it --rm \
    --privileged \
    --network host \
    --ipc=host \
    -v "$WORKSPACE":/ros_ws \
    "$IMAGE" \
    /bin/bash
