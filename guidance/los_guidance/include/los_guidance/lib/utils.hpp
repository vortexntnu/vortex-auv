/**
 * @file yaml_utils.hpp
 * @brief Utility functions for loading YAML configuration files.
 */
#ifndef LOS_GUIDANCE__LIB__UTILS_HPP_
#define LOS_GUIDANCE__LIB__UTILS_HPP_

#include <yaml-cpp/yaml.h>
#include <string>

namespace vortex::guidance::los::utils {

/**
 * @brief Loads a YAML configuration file from the given path.
 * @param yaml_file_path Path to the YAML configuration file.
 * @return YAML::Node Parsed YAML configuration.
 * @throws std::runtime_error if the file cannot be loaded or parsed.
 */
YAML::Node load_yaml_config(const std::string& yaml_file_path);

}  // namespace vortex::guidance::los::utils

#endif  // LOS_GUIDANCE__LIB__UTILS_HPP_
