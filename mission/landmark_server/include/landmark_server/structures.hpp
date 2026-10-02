#ifndef LANDMARK_SERVER__STRUCTURES_HPP_
#define LANDMARK_SERVER__STRUCTURES_HPP_

#include <yaml-cpp/yaml.h>
#include <cstddef>
#include <deque>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>
#include <optional>
#include <pose_filtering/lib/typedefs.hpp>
#include <string>
#include <utility>
#include <vector>

namespace vortex::mission {

struct RetainedLandmark;
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

/**
 * @brief Best placement of a template on free landmarks: hypotheses from
 * every pair that fits two members with a horizontal baseline, the other
 * members by chi-square, yaw and position by weighted least squares, worst
 * members dropped until the whole placement passes chi-square. Needs
 * min_members. With several variants the best must be clearly better.
 */
std::optional<StructureFit> fit_structure(const StructureTemplate& tmpl,
                                          const std::vector<FitLandmark>& free);

/// Refit the yaw and position of a placement to its members.
Eigen::Isometry3d refit_pose(const StructureVariant& variant,
                             const Eigen::Isometry3d& pose,
                             const std::vector<std::pair<std::size_t, FitLandmark>>&
                                 members);

/**
 * @brief The structures in the map (slalom sets ...), kept from tick to
 * tick on top of RetainedLandmarks. ROS-free.
 *
 * Each tick (update): an instance is refitted to its members' current
 * positions (members whose landmark is gone are dropped), landmarks that fit
 * an open slot join it, and free landmarks that fit a template make a new
 * instance.
 *
 * Structure-aware initialisation (queried by RetainedLandmarks for a new
 * track): where a structure stands, its drawing says where the members are.
 * A track near a filled slot, within half the distance to the next member
 * of the drawing, is that member seen badly: it takes that landmark over
 * instead of becoming a new object.
 */
class StructureMap {
   public:
    struct Instance {
        int id{-1};
        StructureTemplate tmpl;
        std::size_t variant{0};
        Eigen::Isometry3d pose{Eigen::Isometry3d::Identity()};
        /// Landmark id per member of the variant, -1 = open.
        std::vector<int> members;
    };

    void set_templates(std::vector<StructureTemplate> templates) {
        templates_ = std::move(templates);
    }

    void update(const std::deque<RetainedLandmark>& landmarks);

    /// The landmark of a filled slot that accepts @p key and lies within the
    /// slot's exclusion radius of @p position, or -1.
    int filled_slot_landmark(const LandmarkClassKey& key,
                             const Eigen::Vector3d& position) const;

    bool in_structure(int landmark_id) const;

    /// Where every member of an instance should be (odom).
    std::vector<Eigen::Vector3d> member_positions(const Instance& s) const;

    const std::vector<Instance>& instances() const { return instances_; }

    void clear() { instances_.clear(); }

   private:
    std::vector<StructureTemplate> templates_;
    std::vector<Instance> instances_;
    int next_id_{0};
};

}  // namespace vortex::mission

#endif  // LANDMARK_SERVER__STRUCTURES_HPP_
