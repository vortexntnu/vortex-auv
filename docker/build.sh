#!/bin/bash
set -e

# ------------------------------------------------------------------------------
# Determine the repository's root path.
# ROOT_PATH is the parent directory of where this script resides.
# ------------------------------------------------------------------------------
ROOT_PATH="$(realpath "$(cd -P "$(dirname "${0}")" && pwd)"/..)"

# Source helper functions from utils.sh (located in the docker directory).
source "${ROOT_PATH}/docker/utils.sh"

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
    )

    # Choose between pushing the image or loading it locally.
    if [[ "${_ENABLE_IMAGE_PUSH:-false}" == "true" ]]; then
      DOCKER_ARGS+=( "--push" )
    else
      DOCKER_ARGS+=( "--load" )
    fi

    # ------------------------------------------------------------------------------
    # Add required build arguments.
    # ------------------------------------------------------------------------------
    DOCKER_ARGS+=( --build-arg "BASE_IMAGE=${BASE_IMAGE}" )
    DOCKER_ARGS+=( --build-arg "COMMAND=${COMMAND}" )

    # ------------------------------------------------------------------------------
    # Function: add_arg_if_set
    # Adds a build argument only if its corresponding environment variable is non-empty.
    # ------------------------------------------------------------------------------
    add_arg_if_set() {
      local var_name="$1"
      local var_value="${!var_name}"
      if [[ -n "${var_value}" ]]; then
        DOCKER_ARGS+=( "--build-arg" "${var_name}=${var_value}" )
      fi
    }

    # ------------------------------------------------------------------------------
    # Add a list of optional build arguments if they are set.
    # These may include additional scripts, dependencies, or configuration.
    # ------------------------------------------------------------------------------
    add_arg_if_set "ADDITIONAL_DEBS_FILE"
    add_arg_if_set "ADDITIONAL_FILES_DIR"
    add_arg_if_set "ADDITIONAL_PIP_FILE"
    add_arg_if_set "AFTER_DEPENDENCY_INSTALLATION_SCRIPT"
    add_arg_if_set "BEFORE_DEPENDENCY_IDENTIFICATION_SCRIPT"
    add_arg_if_set "BEFORE_DEPENDENCY_INSTALLATION_SCRIPT"
    add_arg_if_set "BLACKLISTED_PACKAGES_FILE"
    add_arg_if_set "CMAKE_ARGS"
    add_arg_if_set "CUSTOM_SCRIPT_FILE"
    add_arg_if_set "DISABLE_ROS_INSTALLATION"
    add_arg_if_set "ENABLE_RECURSIVE_ADDITIONAL_DEBS"
    add_arg_if_set "ENABLE_RECURSIVE_ADDITIONAL_PIP"
    add_arg_if_set "ENABLE_RECURSIVE_AFTER_DEPENDENCY_INSTALLATION_SCRIPT"
    add_arg_if_set "ENABLE_RECURSIVE_BEFORE_DEPENDENCY_INSTALLATION_SCRIPT"
    add_arg_if_set "ENABLE_RECURSIVE_BLACKLISTED_PACKAGES"
    add_arg_if_set "ENABLE_RECURSIVE_VCS_IMPORT"
    add_arg_if_set "GIT_HTTPS_PASSWORD"
    add_arg_if_set "GIT_HTTPS_SERVER"
    add_arg_if_set "GIT_HTTPS_USER"
    add_arg_if_set "GIT_SSH_KNOWN_HOST_KEYS"
    add_arg_if_set "GIT_SSH_PRIVATE_KEY"
    add_arg_if_set "RMW_IMPLEMENTATION"
    add_arg_if_set "ROS_DISTRO"
    add_arg_if_set "VCS_IMPORT_FILE"

    # ------------------------------------------------------------------------------
    # Add the custom dependency installation command.
    # ------------------------------------------------------------------------------
    add_arg_if_set "INSTALL_CMD"

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
require_var "BASE_IMAGE"  # Ensure BASE_IMAGE is set.
require_var "IMAGE"       # Ensure IMAGE is set.
[[ "${TARGET}" == *"run"* ]] && require_var "COMMAND"
_ENABLE_IMAGE_PUSH="${_ENABLE_IMAGE_PUSH:-false}"

# ------------------------------------------------------------------------------
# Start the Docker build process.
# ------------------------------------------------------------------------------
build_image
