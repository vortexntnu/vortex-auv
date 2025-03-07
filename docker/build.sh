#!/usr/bin/env bash
set -euo pipefail

# ------------------------------------------------------------------------------
# Set environment variables for the image, base, and platform.
# ------------------------------------------------------------------------------
export IMAGE="auv-image:latest"             # Docker image name/tag
export BASE_IMAGE="ros:humble"     # Base image for Docker builds

# ------------------------------------------------------------------------------
# Detect the target platform.
# On macOS (Darwin), we force 'linux/arm64' (Docker Desktop on Mac).
# Otherwise, use the host's architecture.
# !!!!!! WE FORCE LINUX/ARM64 BECAUSE ORIN USES THIS ARCITECHTURE !!!!!!!!!!!!!!
# ------------------------------------------------------------------------------
export PLATFORM="linux/arm64"

# ------------------------------------------------------------------------------
# Validate the required environment variables.
# ------------------------------------------------------------------------------
: "${BASE_IMAGE:?Environment variable BASE_IMAGE is required}"
: "${IMAGE:?Environment variable IMAGE is required}"

# ------------------------------------------------------------------------------
# Locate this script and the project root (where your Dockerfile context lives).
# ------------------------------------------------------------------------------
SCRIPT_DIR="$(dirname "$(realpath "$0")")"
WORKSPACE="$(realpath "$SCRIPT_DIR/../../..")"

echo "======================================================================"
echo " Building Docker image"
echo "   * PLATFORM:       $PLATFORM"
echo "   * BASE_IMAGE:     $BASE_IMAGE"
echo "   * IMAGE:          $IMAGE"
echo "   * BUILD CONTEXT:  $WORKSPACE"
echo "======================================================================"
echo ""

# ------------------------------------------------------------------------------
# Build the Docker image using Docker Buildx.
# ------------------------------------------------------------------------------
docker buildx build \
    --platform "$PLATFORM" \
    --build-arg BASE_IMAGE="$BASE_IMAGE" \
    --tag "$IMAGE" \
    --file "$SCRIPT_DIR/Dockerfile" \
    --load \
    "$WORKSPACE"

echo ""
echo "======================================================================"
echo "Successfully built image '$IMAGE' for platform '$PLATFORM'"
echo "======================================================================"
