#!/bin/bash
set -e

# ------------------------------------------------------------------------------
# build_image: Function to build the Docker image.
# ------------------------------------------------------------------------------
build_image() {
    echo "Building for platform '${PLATFORM}' as '${IMAGE}'..."

    # Construct an array of Docker build arguments.
    DOCKER_ARGS=(
      --file "$(dirname "$0")/../docker/Dockerfile"  # Dockerfile location
      --platform "${PLATFORM}"                        # Target platform (e.g., linux/arm64)
      --tag "${IMAGE}"                                # Tag for the built image
      --load                                          # Load the image locally
      "."
    )

    # Run the Docker build command
    docker buildx build "${DOCKER_ARGS[@]}"

    echo "Successfully built for platform '${PLATFORM}' as '${IMAGE}'"
}

# ------------------------------------------------------------------------------
# Validate environment variables and set defaults.
# ------------------------------------------------------------------------------
PLATFORM="${PLATFORM:-$(dpkg --print-architecture)}"
: "${BASE_IMAGE:?Environment variable BASE_IMAGE is required}"
: "${IMAGE:?Environment variable IMAGE is required}"

# ------------------------------------------------------------------------------
# Start the Docker build process.
# ------------------------------------------------------------------------------
build_image
