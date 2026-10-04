#ifndef LANDMARK_SERVER__STRUCTURES_HPP_
#define LANDMARK_SERVER__STRUCTURES_HPP_

#include <yaml-cpp/yaml.h>
#include <cmath>
#include <cstddef>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>
#include <limits>
#include <optional>
#include <pose_filtering/lib/typedefs.hpp>
#include <string>
#include <utility>
#include <vector>

namespace vortex::mission {

using vortex::filtering::LandmarkClassKey;

/// One part of a structure in the structure frame (+X out of the front, +Y
/// right, +Z down).
struct StructureMember {
    std::string name;
    /// The classes this part can be detected as ({type, 0}: any subtype).
    std::vector<LandmarkClassKey> classes;
    Eigen::Vector3d offset{Eigen::Vector3d::Zero()};
    /// How well the prop and the detector follow the drawing [m], per axis.
    Eigen::Vector3d sigma{0.2, 0.2, 0.3};

    bool accepts(const LandmarkClassKey& key) const;
};

struct StructureVariant {
    std::string name;
    std::vector<StructureMember> members;
};

/**
 * @brief A rigid arrangement of landmark classes from the task drawings
 * (config `rules.structures`), e.g. a slalom set: white, red, white 1.52 m
 * apart in a line.
 */
struct StructureTemplate {
    std::string name;
    int max_instances{1};
    /// Members that must fit before an instance is made.
    int min_members{2};
    /// A member fits when its chi-square (3 dof) is below this.
    double member_gate_chi2{11.34};
    /// The placement as a whole must pass chi-square at this probability.
    double fit_probability{0.99};
    /// With several variants, the best must be this much better (chi-square).
    double variant_margin_chi2{6.0};
    /// The parts that allow several classes use each of them equally often
    /// (one Search & Rescue and one Survey & Repair panel, two bins of each
    /// role, each octagon image once): the classes are decided together.
    bool balanced_classes{false};
    std::vector<StructureVariant> variants;

    bool has_member_class(const LandmarkClassKey& key) const;
};

/**
 * @brief Parse `rules.structures`: a map of name -> {max_instances,
 * min_members, sigma, members: {name: {class, offset, sigma}}} (or variants:
 * {name: {members}}). Class names as in `classes` (a type or a full subtype
 * constant).
 * @throws std::runtime_error with the key on errors.
 */
std::vector<StructureTemplate> parse_structures(const YAML::Node& node);

/// Chi-square quantile (probability p, dof degrees of freedom).
double chi2_quantile(double p, int dof);

/// A mapped landmark offered to a structure (odom frame).
struct FitLandmark {
    int id{-1};
    LandmarkClassKey key;
    Eigen::Vector3d position{Eigen::Vector3d::Zero()};
    Eigen::Matrix3d covariance{Eigen::Matrix3d::Identity() * 0.01};
};

/// A template placed on landmarks.
struct StructureFit {
    std::size_t variant{0};
    /// odom_T_structure, a yaw rotation (props stand upright).
    Eigen::Isometry3d pose{Eigen::Isometry3d::Identity()};
    /// (member index, landmark id).
    std::vector<std::pair<std::size_t, int>> members;
    double cost{0.0};
};

double member_chi2(const StructureMember& m,
                   const Eigen::Isometry3d& pose,
                   const FitLandmark& lm);

/// Where a placement may lie: yaw within a window around a prior yaw (also
/// turned 180 deg for a template that looks the same both ways), position
/// (xy) within a radius of a prior point.
struct FitPrior {
    double yaw{0.0};
    double yaw_window{M_PI};
    bool symmetric{false};
    Eigen::Vector2d center{Eigen::Vector2d::Zero()};
    double radius{std::numeric_limits<double>::infinity()};

    bool allows(const Eigen::Isometry3d& pose) const;
};

/**
 * @brief Best placement of a template on free landmarks: hypotheses from
 * every pair that fits two members with a horizontal baseline, the other
 * members by chi-square, yaw and position by weighted least squares, worst
 * members dropped until the whole placement passes chi-square. Needs
 * min_members (or @p min_members when > 0). With several variants the best
 * must be clearly better. With a prior, placements outside it are not
 * considered.
 */
std::optional<StructureFit> fit_structure(
    const StructureTemplate& tmpl,
    const std::vector<FitLandmark>& free,
    const std::optional<FitPrior>& prior = std::nullopt,
    int min_members = 0);

/// Refit the yaw and position of a placement to its members.
Eigen::Isometry3d refit_pose(const StructureVariant& variant,
                             const Eigen::Isometry3d& pose,
                             const std::vector<std::pair<std::size_t, FitLandmark>>&
                                 members);

}  // namespace vortex::mission

#endif  // LANDMARK_SERVER__STRUCTURES_HPP_
