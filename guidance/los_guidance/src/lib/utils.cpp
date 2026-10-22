#include "los_guidance/lib/utils.hpp"
#include <stdexcept>

namespace vortex::guidance::los::utils {

YAML::Node load_yaml_config(const std::string& yaml_file_path) {
    try {
        YAML::Node config = YAML::LoadFile(yaml_file_path);
        return config;
    } catch (const YAML::Exception& e) {
        throw std::runtime_error(
            std::string("Failed to load LOS config file '") + yaml_file_path +
            "': " + e.what());
    }
}

}  // namespace vortex::guidance::los::utils
