#include <gtest/gtest.h>

#include <Eigen/Dense>
#include <cmath>

#include "line_filtering/lib/line_track_manager.hpp"

namespace vortex::line_filtering {

class LineTrackManagerTests : public ::testing::Test {
   protected:
    LineTrackManagerConfig make_default_config() {
        LineTrackManagerConfig cfg{};
        cfg.default_config.dyn_std_dev = 0.05;
        cfg.default_config.sens_std_dev = 0.1;
        cfg.default_config.init_rho_std = 0.3;
        cfg.default_config.init_phi_std = 0.15;
        cfg.default_config.max_rho_error = 1.0;
        cfg.default_config.max_phi_error = 0.5;
        cfg.default_config.mahalanobis_threshold = 3.0;
        cfg.existence.initial_existence_probability = 0.5;
        cfg.existence.confirmation_threshold = 0.6;
        cfg.existence.deletion_threshold = 0.2;
        return cfg;
    }

    LineMeasurement make_measurement(double rho, double theta) {
        return LineMeasurement{
            .rho = rho,
            .n = Eigen::Vector2d{std::cos(theta), std::sin(theta)},
        };
    }
};

TEST_F(LineTrackManagerTests, creates_tracks_from_measurements) {
    LineTrackManager mgr(make_default_config());

    std::vector<LineMeasurement> measurements{
        make_measurement(1.0, 0.0),
        make_measurement(2.0, M_PI / 2),
        make_measurement(3.0, M_PI / 4),
    };

    mgr.step(measurements, 0.1);

    const auto& tracks = mgr.get_tracks();
    ASSERT_EQ(tracks.size(), 3);

    for (const auto& t : tracks) {
        EXPECT_FALSE(t.confirmed);
        EXPECT_NEAR(t.existence_probability, 0.5, 1e-12);
    }
}

TEST_F(LineTrackManagerTests, single_target_existence_increase) {
    auto cfg = make_default_config();
    LineTrackManager mgr(cfg);

    std::vector<LineMeasurement> z = {make_measurement(1.0, 0.0)};
    mgr.step(z, 0.1);

    ASSERT_EQ(mgr.get_tracks().size(), 1);
    double exist_prob1 = mgr.get_tracks().front().existence_probability;
    EXPECT_NEAR(exist_prob1, 0.5, 1e-12);

    // Repeated observation at same location should increase existence
    z = {make_measurement(1.0, 0.0)};
    mgr.step(z, 0.1);

    ASSERT_EQ(mgr.get_tracks().size(), 1);
    double exist_prob2 = mgr.get_tracks().front().existence_probability;
    EXPECT_GT(exist_prob2, exist_prob1);

    // Third observation
    z = {make_measurement(1.0, 0.0)};
    mgr.step(z, 0.1);

    ASSERT_EQ(mgr.get_tracks().size(), 1);
    double exist_prob3 = mgr.get_tracks().front().existence_probability;
    EXPECT_GT(exist_prob3, exist_prob2);
}

TEST_F(LineTrackManagerTests, existence_decrease_on_no_detection) {
    auto cfg = make_default_config();
    LineTrackManager mgr(cfg);

    std::vector<LineMeasurement> z = {make_measurement(1.0, 0.0)};
    mgr.step(z, 0.1);

    ASSERT_EQ(mgr.get_tracks().size(), 1);
    double exist_prob1 = mgr.get_tracks().front().existence_probability;

    // Step with no measurements — existence should drop
    z = {};
    mgr.step(z, 0.1);

    const auto& tracks_after = mgr.get_tracks();
    if (tracks_after.empty()) {
        SUCCEED();  // track was deleted due to low existence
    } else {
        double exist_prob2 = tracks_after.front().existence_probability;
        EXPECT_LT(exist_prob2, exist_prob1);
    }
}

TEST_F(LineTrackManagerTests, rho_gate_filters_distant_measurement) {
    auto cfg = make_default_config();
    cfg.default_config.max_rho_error = 0.5;  // tight rho gate
    LineTrackManager mgr(cfg);

    // Create a track at rho=1.0, theta=0
    std::vector<LineMeasurement> z1 = {make_measurement(1.0, 0.0)};
    mgr.step(z1, 0.1);
    ASSERT_EQ(mgr.get_tracks().size(), 1);

    // Measurement far away in rho (rho=5.0) should not associate,
    // creating a second track instead
    std::vector<LineMeasurement> z2 = {make_measurement(5.0, 0.0)};
    mgr.step(z2, 0.1);

    // The distant measurement should have created a new track
    EXPECT_GE(mgr.get_tracks().size(), 2u);
}

TEST_F(LineTrackManagerTests, phi_gate_filters_rotated_measurement) {
    auto cfg = make_default_config();
    cfg.default_config.max_phi_error = 0.2;  // tight angle gate (radians)
    LineTrackManager mgr(cfg);

    // Create a track with theta=0
    std::vector<LineMeasurement> z1 = {make_measurement(1.0, 0.0)};
    mgr.step(z1, 0.1);
    ASSERT_EQ(mgr.get_tracks().size(), 1);

    // Measurement rotated by 1.0 rad should not associate through the gate
    std::vector<LineMeasurement> z2 = {make_measurement(1.0, 1.0)};
    mgr.step(z2, 0.1);

    EXPECT_GE(mgr.get_tracks().size(), 2u);
}

TEST_F(LineTrackManagerTests, sign_alignment_handles_flipped_normal) {
    auto cfg = make_default_config();
    LineTrackManager mgr(cfg);

    // Create track with (rho=1.0, n=[1, 0])
    std::vector<LineMeasurement> z1 = {make_measurement(1.0, 0.0)};
    mgr.step(z1, 0.1);

    ASSERT_EQ(mgr.get_tracks().size(), 1);
    double exist_prob1 = mgr.get_tracks().front().existence_probability;

    // Equivalent line with flipped sign: (rho=-1.0, n=[-1, 0])
    // This should be sign-aligned and associated to the same track
    std::vector<LineMeasurement> z2 = {make_measurement(-1.0, M_PI)};
    mgr.step(z2, 0.1);

    // Should still be one track (the flipped measurement was associated)
    ASSERT_EQ(mgr.get_tracks().size(), 1);
    double exist_prob2 = mgr.get_tracks().front().existence_probability;
    EXPECT_GT(exist_prob2, exist_prob1);
}

TEST_F(LineTrackManagerTests, track_confirmed_after_repeated_observations) {
    auto cfg = make_default_config();
    cfg.existence.confirmation_threshold = 0.6;
    LineTrackManager mgr(cfg);

    // Feed the same measurement many times to push existence past threshold
    for (int i = 0; i < 20; ++i) {
        std::vector<LineMeasurement> z = {make_measurement(1.0, 0.0)};
        mgr.step(z, 0.1);
    }

    ASSERT_EQ(mgr.get_tracks().size(), 1);
    EXPECT_TRUE(mgr.get_tracks().front().confirmed);
    EXPECT_GE(mgr.get_tracks().front().existence_probability,
              cfg.existence.confirmation_threshold);
}

TEST_F(LineTrackManagerTests, track_deleted_when_existence_drops) {
    auto cfg = make_default_config();
    cfg.existence.deletion_threshold = 0.3;
    LineTrackManager mgr(cfg);

    // Create a track
    std::vector<LineMeasurement> z = {make_measurement(1.0, 0.0)};
    mgr.step(z, 0.1);
    ASSERT_EQ(mgr.get_tracks().size(), 1);

    // Many empty steps should eventually delete the track
    for (int i = 0; i < 50; ++i) {
        z = {};
        mgr.step(z, 0.1);
    }

    EXPECT_EQ(mgr.get_tracks().size(), 0);
}

TEST_F(LineTrackManagerTests, multiple_tracks_maintained_independently) {
    auto cfg = make_default_config();
    LineTrackManager mgr(cfg);

    // Create two tracks far apart
    std::vector<LineMeasurement> z = {
        make_measurement(1.0, 0.0),
        make_measurement(5.0, M_PI / 2),
    };
    mgr.step(z, 0.1);
    ASSERT_EQ(mgr.get_tracks().size(), 2);

    // Observe only the first track
    z = {make_measurement(1.0, 0.0)};
    mgr.step(z, 0.1);

    // Both tracks should still exist, first with higher probability
    ASSERT_GE(mgr.get_tracks().size(), 1u);

    // Find the track closest to rho=1.0, theta=0
    bool found_first = false;
    for (const auto& t : mgr.get_tracks()) {
        if (std::abs(t.nominal.rho - 1.0) < 0.5 &&
            std::abs(t.nominal.n.x() - 1.0) < 0.5) {
            found_first = true;
            EXPECT_GT(t.existence_probability, 0.5);
        }
    }
    EXPECT_TRUE(found_first);
}

TEST_F(LineTrackManagerTests, from_segment_creates_valid_measurement) {
    LineSegment2D seg;
    seg.p0 = {0.0, 0.0};
    seg.p1 = {1.0, 0.0};

    auto meas = LineMeasurement::from_segment(seg);

    // Segment along x-axis: normal is [0, 1], rho = 0
    EXPECT_NEAR(std::abs(meas.n.x()), 0.0, 1e-12);
    EXPECT_NEAR(std::abs(meas.n.y()), 1.0, 1e-12);
    EXPECT_NEAR(meas.rho, 0.0, 1e-12);
    EXPECT_NEAR(meas.n.norm(), 1.0, 1e-12);
}

TEST_F(LineTrackManagerTests, from_segment_diagonal_line) {
    LineSegment2D seg;
    seg.p0 = {0.0, 0.0};
    seg.p1 = {1.0, 1.0};

    auto meas = LineMeasurement::from_segment(seg);

    // Unit normal should have magnitude 1
    EXPECT_NEAR(meas.n.norm(), 1.0, 1e-12);

    // rho = n . p0 = 0 (line passes through origin)
    EXPECT_NEAR(meas.rho, 0.0, 1e-12);
}

TEST_F(LineTrackManagerTests, from_segment_offset_line) {
    LineSegment2D seg;
    seg.p0 = {0.0, 2.0};
    seg.p1 = {1.0, 2.0};

    auto meas = LineMeasurement::from_segment(seg);

    // Horizontal line at y=2: normal is [0, ±1], |rho| = 2
    EXPECT_NEAR(std::abs(meas.n.x()), 0.0, 1e-12);
    EXPECT_NEAR(std::abs(meas.n.y()), 1.0, 1e-12);
    EXPECT_NEAR(std::abs(meas.rho), 2.0, 1e-12);
}

TEST_F(LineTrackManagerTests, from_segment_zero_length_throws) {
    LineSegment2D seg;
    seg.p0 = {1.0, 1.0};
    seg.p1 = {1.0, 1.0};

    EXPECT_THROW(LineMeasurement::from_segment(seg), std::runtime_error);
}

TEST_F(LineTrackManagerTests, canonical_form_positive_rho) {
    LineTrackManager mgr(make_default_config());

    // Create track with negative rho
    std::vector<LineMeasurement> z = {make_measurement(-2.0, M_PI)};
    mgr.step(z, 0.1);

    ASSERT_EQ(mgr.get_tracks().size(), 1);

    auto canonical = mgr.get_tracks().front().to_canonical_line();
    EXPECT_GE(canonical.rho, 0.0);
    EXPECT_GE(canonical.theta, 0.0);
    EXPECT_LT(canonical.theta, 2.0 * M_PI);
}

}  // namespace vortex::line_filtering
