#!/bin/bash
set -e

# ------------------------------------------------------------------------------
# build_image: Function to build the Docker image.
# ------------------------------------------------------------------------------
build_image() {
    echo "Building stage '${TARGET}' for platform '${PLATFORM}' as '${IMAGE}' ..."

    # Construct an array of Docker build arguments.
    DOCKER_ARGS=(
      --file "$(dirname "$0")/../docker/Dockerfile"  # Specify the Dockerfile location.
      --target "${TARGET}"                            # Target build stage.
      --platform "${PLATFORM}"                        # Target platform (e.g., linux/arm64).
      --tag "${IMAGE}"                                # Tag for the built image.
      --load                                          # Always load the image locally
    )

    # Use the current directory as the build context.
    DOCKER_ARGS+=( "." )

    # ------------------------------------------------------------------------------
    # Run the Docker build command with all the assembled arguments.
    # ------------------------------------------------------------------------------
    docker buildx build "${DOCKER_ARGS[@]}"
    echo "Successfully built stage '${TARGET}' for platform '${PLATFORM}' as '${IMAGE}'"
}

# ------------------------------------------------------------------------------
# Validate required environment variables and set defaults.
# ------------------------------------------------------------------------------
TARGET="${TARGET:-run}"
PLATFORM="${PLATFORM:-$(dpkg --print-architecture)}"
: "${BASE_IMAGE:?Environment variable BASE_IMAGE is required}"
: "${IMAGE:?Environment variable IMAGE is required}"

# ------------------------------------------------------------------------------
# Start the Docker build process.
# ------------------------------------------------------------------------------
build_image
