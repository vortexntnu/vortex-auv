#include <gtest/gtest.h>
#include <cmath>
#include <stdexcept>
#include <vortex_msgs/msg/landmark_subtype.hpp>
#include <vortex_msgs/msg/landmark_type.hpp>
#include "landmark_server/structures.hpp"

namespace vortex::mission {

namespace {

using LT = vortex_msgs::msg::LandmarkType;
using LS = vortex_msgs::msg::LandmarkSubtype;

/// A slalom set: white, red, white 1.52 m apart.
StructureTemplate slalom_template() {
    return parse_structures(YAML::Load(R"(
slalom_set:
  min_members: 2
  sigma: [0.2, 0.2, 0.3]
  members:
    white_left: {class: SLALOM_PIPE_WHITE, offset: [0.0, -1.52, 0.0]}
    red: {class: SLALOM_PIPE_RED, offset: [0.0, 0.0, 0.0]}
    white_right: {class: SLALOM_PIPE_WHITE, offset: [0.0, 1.52, 0.0]}
)"))[0];
}

Eigen::Vector3d v(double x, double y, double z = 2.6) { return {x, y, z}; }

FitLandmark pipe(int id, uint16_t subtype, const Eigen::Vector3d& p) {
    FitLandmark f;
    f.id = id;
    f.key = {LT::SLALOM_PIPE, subtype};
    f.position = p;
    f.covariance = Eigen::Matrix3d::Identity() * 0.01;
    return f;
}

}  // namespace

TEST(Structures, TheSlalomTemplateParses) {
    const auto t = slalom_template();
    EXPECT_EQ(t.name, "slalom_set");
    ASSERT_EQ(t.variants[0].members.size(), 3u);
    EXPECT_TRUE(t.has_member_class({LT::SLALOM_PIPE, LS::SLALOM_PIPE_RED}));
    EXPECT_FALSE(t.has_member_class({LT::GATE, LS::GATE_WHOLE}));
}

TEST(Structures, MistakesAreRejectedWithTheKey) {
    EXPECT_THROW(parse_structures(YAML::Load(R"(
s: {members: {a: {class: SLALOM_PIPE_PINK, offset: [0, 0, 0]}}}
)")),
                 std::runtime_error);
    EXPECT_THROW(parse_structures(YAML::Load(R"(
s: {members: {a: {class: SLALOM_PIPE_RED, offset: [0, 0]},
              b: {class: SLALOM_PIPE_WHITE}}}
)")),
                 std::runtime_error);
}

TEST(Structures, ASetFromItsPipes) {
    const std::vector<FitLandmark> lms = {
        pipe(0, LS::SLALOM_PIPE_WHITE, v(8.0, -1.4)),
        pipe(1, LS::SLALOM_PIPE_RED, v(8.1, 0.1)),
        pipe(2, LS::SLALOM_PIPE_WHITE, v(8.0, 1.65)),
        // the next set's white, 2 m on
        pipe(3, LS::SLALOM_PIPE_WHITE, v(10.0, -1.0))};
    const auto fit = fit_structure(slalom_template(), lms);
    ASSERT_TRUE(fit);
    EXPECT_EQ(fit->members.size(), 3u);
    for (const auto& [m, id] : fit->members) {
        EXPECT_NE(id, 3);
    }
    EXPECT_LT((fit->pose.translation() - v(8.07, 0.1)).head<2>().norm(), 0.15);
}

TEST(Structures, APriorKeepsTheSetWhereAndHowTheCourseSays) {
    // A white and a red at the right spacing, but lying along the course
    // (the sets lie across it): no placement within 20 deg of the prior.
    const std::vector<FitLandmark> along = {
        pipe(0, LS::SLALOM_PIPE_WHITE, v(6.5, 0.0)),
        pipe(1, LS::SLALOM_PIPE_RED, v(8.0, 0.0))};
    FitPrior prior;
    prior.yaw = 0.0;
    prior.yaw_window = 20.0 * M_PI / 180.0;
    prior.symmetric = true;
    prior.center = {8.0, 0.0};
    prior.radius = 1.5;
    EXPECT_FALSE(fit_structure(slalom_template(), along, prior));

    // Across the course: placed, also turned 180 deg (the set is symmetric).
    const std::vector<FitLandmark> across = {
        pipe(0, LS::SLALOM_PIPE_WHITE, v(8.0, 1.52)),
        pipe(1, LS::SLALOM_PIPE_RED, v(8.0, 0.0))};
    EXPECT_TRUE(fit_structure(slalom_template(), across, prior));

    // Outside the region.
    prior.center = {12.0, 0.0};
    EXPECT_FALSE(fit_structure(slalom_template(), across, prior));
}

TEST(Structures, MorePartsCanBeRequired) {
    const std::vector<FitLandmark> two = {
        pipe(0, LS::SLALOM_PIPE_WHITE, v(8.0, 1.52)),
        pipe(1, LS::SLALOM_PIPE_RED, v(8.0, 0.0))};
    EXPECT_TRUE(fit_structure(slalom_template(), two, std::nullopt, 2));
    EXPECT_FALSE(fit_structure(slalom_template(), two, std::nullopt, 3));
}

}  // namespace vortex::mission
