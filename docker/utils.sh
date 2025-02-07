#!/bin/bash

# ------------------------------------------------------------------------------
# require_var: Checks if an environment variable is set; if not, exits with an error.
# Usage: require_var VARIABLE_NAME
# ------------------------------------------------------------------------------
require_var() {
    if [[ -z "${!1}" ]]; then
        echo "Environment variable '${1}' is required"
        exit 1
    fi
}

# ------------------------------------------------------------------------------
# open_log_group: Outputs a log group header for CI systems like GitLab CI or GitHub Actions.
# ------------------------------------------------------------------------------
open_log_group() {
    if [[ -n "${GITLAB_CI}" ]]; then
        echo -e "section_start:`date +%s`:build_section[collapsed=true]\r\e[0K[docker] ${1}"
    elif [[ -n "${GITHUB_ACTIONS}" ]]; then
        echo "::group::[docker] ${1}"
    fi
}

# ------------------------------------------------------------------------------
# close_log_group: Outputs a log group footer for CI systems.
# ------------------------------------------------------------------------------
close_log_group() {
    if [[ -n "${GITLAB_CI}" ]]; then
        echo -e "section_end:`date +%s`:build_section\r\e[0K"
    elif [[ -n "${GITHUB_ACTIONS}" ]]; then
        echo "::endgroup::"
    fi
}
