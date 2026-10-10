#include "landmark_server/premap.hpp"

#include <fmt/format.h>

#include <chrono>
#include <cmath>
#include <ctime>
#include <filesystem>
#include <fstream>
#include <stdexcept>

namespace vortex::landmark_server {

namespace {

constexpr const char* kMapReference = "start";

std::string format_time(std::time_t t, const char* format) {
    char buf[32];
    std::strftime(buf, sizeof(buf), format, std::localtime(&t));
    return buf;
}

std::string backup_stamp(const std::string& created_at,
                         const std::filesystem::path& path) {
    std::tm tm{};
    tm.tm_isdst = -1;
    if (!created_at.empty() &&
        strptime(created_at.c_str(), "%Y-%m-%dT%H:%M:%S", &tm) != nullptr) {
        return format_time(std::mktime(&tm), "%Y%m%d_%H%M");
    }
    const auto ftime = std::filesystem::last_write_time(path);
    const auto sys =
        std::chrono::time_point_cast<std::chrono::system_clock::duration>(
            ftime - std::filesystem::file_time_type::clock::now() +
            std::chrono::system_clock::now());
    return format_time(std::chrono::system_clock::to_time_t(sys),
                       "%Y%m%d_%H%M");
}

}  // namespace

std::map<std::string, gtsam::Point3> Premap::positions() const {
    std::map<std::string, gtsam::Point3> out;
    for (const auto& [label, pose] : objects) {
        out[label] = pose.translation();
    }
    return out;
}

bool is_map_reference(const std::string& frame) {
    return frame.empty() || frame == kMapReference || frame == "map";
}

std::string now_iso8601() {
    return format_time(std::time(nullptr), "%Y-%m-%dT%H:%M:%S");
}

Premap load_premap(const std::string& path, std::vector<std::string>& skipped) {
    if (!std::filesystem::exists(path)) {
        throw std::runtime_error(fmt::format("no such file: '{}'", path));
    }
    const YAML::Node root = YAML::LoadFile(path);
    Premap premap;
    if (!root || root.IsNull()) {
        return premap;
    }
    const auto reference = root["reference_frame"]
                               ? root["reference_frame"].as<std::string>()
                               : std::string();
    if (!is_map_reference(reference)) {
        throw std::runtime_error(fmt::format(
            "{}: reference_frame '{}': a file must be in the start frame "
            "(send poses in another frame through set_premap instead)",
            path, reference));
    }
    if (root["created_at"]) {
        premap.created_at = root["created_at"].as<std::string>();
    }
    if (root["gui_state"]) {
        premap.gui_state = root["gui_state"];
    }
    for (const auto& entry : root["objects"]) {
        const auto label = entry.first.as<std::string>();
        const YAML::Node& n = entry.second;
        try {
            const auto p = n["position"].as<std::vector<double>>();
            const auto q = n["orientation"]
                               ? n["orientation"].as<std::vector<double>>()
                               : std::vector<double>{0.0, 0.0, 0.0, 1.0};
            const double norm = q.size() == 4
                                    ? std::sqrt(q[0] * q[0] + q[1] * q[1] +
                                                q[2] * q[2] + q[3] * q[3])
                                    : 0.0;
            if (p.size() != 3 || norm < 1e-8 || !std::isfinite(p[0]) ||
                !std::isfinite(p[1]) || !std::isfinite(p[2])) {
                skipped.push_back(label);
                continue;
            }
            premap.objects[label] =
                gtsam::Pose3(gtsam::Rot3::Quaternion(q[3] / norm, q[0] / norm,
                                                     q[1] / norm, q[2] / norm),
                             gtsam::Point3(p[0], p[1], p[2]));
            if (n["radius"] && n["radius"].as<double>() > 0.0) {
                premap.radius[label] = n["radius"].as<double>();
            }
        } catch (const YAML::Exception&) {
            skipped.push_back(label);
        }
    }
    return premap;
}

std::string premap_to_yaml(const Premap& premap) {
    // Avoid writing -0 and 6e-17.
    const auto clean = [](std::vector<double> v) {
        for (double& x : v) {
            x = std::abs(x) < 1e-9 ? 0.0 : x;
        }
        return v;
    };
    YAML::Emitter out;
    out.SetDoublePrecision(7);
    out << YAML::BeginMap;
    out << YAML::Key << "reference_frame" << YAML::Value << kMapReference;
    out << YAML::Key << "created_at" << YAML::Value << YAML::SingleQuoted
        << premap.created_at;
    out << YAML::Key << "objects" << YAML::Value << YAML::BeginMap;
    for (const auto& [label, pose] : premap.objects) {
        const gtsam::Quaternion q = pose.rotation().toQuaternion();
        out << YAML::Key << label << YAML::Value << YAML::BeginMap;
        out << YAML::Key << "position" << YAML::Value << YAML::Flow
            << clean({pose.x(), pose.y(), pose.z()});
        out << YAML::Key << "orientation" << YAML::Value << YAML::Flow
            << clean({q.x(), q.y(), q.z(), q.w()});
        if (const auto r = premap.radius.find(label);
            r != premap.radius.end()) {
            out << YAML::Key << "radius" << YAML::Value << r->second;
        }
        out << YAML::EndMap;
    }
    out << YAML::EndMap;
    if (premap.gui_state && !premap.gui_state.IsNull()) {
        out << YAML::Key << "gui_state" << YAML::Value << premap.gui_state;
    }
    out << YAML::EndMap;
    return std::string(out.c_str()) + "\n";
}

std::string save_premap(const std::string& path, const Premap& premap) {
    namespace fs = std::filesystem;
    // With --symlink-install, write the source file.
    const fs::path file = fs::exists(path) && fs::is_symlink(path)
                              ? fs::canonical(path)
                              : fs::path(path);
    const std::string target = file.string();
    if (file.has_parent_path()) {
        fs::create_directories(file.parent_path());
    }
    std::string backup;
    if (fs::exists(file)) {
        std::string old_created;
        try {
            const YAML::Node old = YAML::LoadFile(target);
            if (old && old["created_at"]) {
                old_created = old["created_at"].as<std::string>();
            }
        } catch (const YAML::Exception&) {
        }
        backup = (file.parent_path() /
                  (file.stem().string() + "_" +
                   backup_stamp(old_created, file) + file.extension().string()))
                     .string();
        fs::rename(file, backup);
    }
    std::ofstream f(target);
    if (!f) {
        throw std::runtime_error(fmt::format("cannot write '{}'", target));
    }
    f << premap_to_yaml(premap);
    return backup;
}

}  // namespace vortex::landmark_server
