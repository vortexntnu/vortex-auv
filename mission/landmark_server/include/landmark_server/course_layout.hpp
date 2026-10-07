#ifndef LANDMARK_SERVER__COURSE_LAYOUT_HPP_
#define LANDMARK_SERVER__COURSE_LAYOUT_HPP_

#include <yaml-cpp/yaml.h>
#include <filesystem>
#include <string>
#include <vector>

namespace vortex::mission {

/**
 * @brief Course layouts: where the tasks are, one file per pool or
 * competition course (config/course/<layout>.yaml beside templates.yaml).
 * ROS-free.
 *
 * A layout file is a ROS parameter file with `course.enable`,
 * `course.start` and `course.tasks` (and any other `course` key it sets on
 * top of templates.yaml), plus `course_gui_state`: the operator GUI's
 * canvas, stored and handed back as it is.
 */

/// One task as the operator places it. Course frame: origin at the gate,
/// x through the gate away from the start, y right.
struct LayoutTask {
    std::string name;
    std::string template_name;
    double x{0.0};
    double y{0.0};
    /// Where the front of the prop faces (task +X), from +x towards +y.
    double yaw_deg{0.0};
    /// 0 = the template's.
    double region_radius_m{0.0};
};

struct CourseLayout {
    bool enable{false};
    double start_x{0.0};
    double start_y{0.0};
    std::vector<LayoutTask> tasks;
};

/// A layout name: letters, digits, _ and -, and not "templates".
bool valid_layout_name(const std::string& name);

/// The layouts in a directory: every .yaml file but templates.yaml, sorted.
std::vector<std::string> list_layouts(const std::filesystem::path& dir);

/// Whether files saved in this layout directory land in the package source,
/// as in a workspace built with --symlink-install: the package source has
/// CMakeLists.txt next to config/, the install space has not.
bool in_source_tree(const std::filesystem::path& layout_dir);

/// The `course` tree and the GUI state of a parameter file.
struct LayoutFile {
    YAML::Node course;
    std::string gui_state;
};

/// @throws std::runtime_error when the file cannot be read or has no
/// `course` parameters.
LayoutFile read_layout_file(const std::filesystem::path& file);

/**
 * @brief The `course` tree of another layout: @p base without its layout
 * keys (enable, start, tasks), with @p layout_course merged on top the way
 * ROS merges parameter files (maps key by key, the rest replaced).
 */
YAML::Node with_layout_file(const YAML::Node& base,
                            const YAML::Node& layout_course);

/// The region radius the tasks of a template get when they set none [m].
double template_region_radius(const YAML::Node& course,
                              const std::string& template_name);

/**
 * @brief The `course` tree with the enable, start and tasks of @p layout.
 * A task already in @p course with the same template keeps its other keys
 * (its own tolerances); region_radius_m is written only where it differs
 * from the template's. Numbers are rounded to mm and 0.01 deg.
 * @throws std::runtime_error on a task without a name or template, a name
 * used twice, or a value that is not finite.
 */
YAML::Node apply_layout(const YAML::Node& course, const CourseLayout& layout);

/**
 * @brief Write a layout file: enable, start and tasks from @p course, the
 * GUI state, and whatever else the file had. An existing file is first
 * copied to backup/<layout>_<timestamp>.yaml next to it.
 * @return The backup's path; empty when there was no file.
 * @throws std::runtime_error when a file cannot be written.
 */
std::filesystem::path write_layout_file(const std::filesystem::path& file,
                                        const YAML::Node& course,
                                        const std::string& gui_state,
                                        const std::string& timestamp);

}  // namespace vortex::mission

#endif  // LANDMARK_SERVER__COURSE_LAYOUT_HPP_
