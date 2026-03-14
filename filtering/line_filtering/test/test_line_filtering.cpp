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
        cfg.nm.confirm_n = 3;
        cfg.nm.confirm_m = 5;
        cfg.nm.delete_n = 5;
        cfg.nm.delete_m = 7;
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
        EXPECT_EQ(t.hits(), 1);
    }
}

TEST_F(LineTrackManagerTests, hits_increase_with_repeated_observation) {
    auto cfg = make_default_config();
    LineTrackManager mgr(cfg);

    std::vector<LineMeasurement> z = {make_measurement(1.0, 0.0)};
    mgr.step(z, 0.1);

    ASSERT_EQ(mgr.get_tracks().size(), 1);
    EXPECT_EQ(mgr.get_tracks().front().hits(), 1);

    z = {make_measurement(1.0, 0.0)};
    mgr.step(z, 0.1);

    ASSERT_EQ(mgr.get_tracks().size(), 1);
    EXPECT_EQ(mgr.get_tracks().front().hits(), 2);

    z = {make_measurement(1.0, 0.0)};
    mgr.step(z, 0.1);

    ASSERT_EQ(mgr.get_tracks().size(), 1);
    EXPECT_EQ(mgr.get_tracks().front().hits(), 3);
}

TEST_F(LineTrackManagerTests, misses_increase_on_no_detection) {
    auto cfg = make_default_config();
    LineTrackManager mgr(cfg);

    std::vector<LineMeasurement> z = {make_measurement(1.0, 0.0)};
    mgr.step(z, 0.1);

    ASSERT_EQ(mgr.get_tracks().size(), 1);

    z = {};
    mgr.step(z, 0.1);

    const auto& tracks_after = mgr.get_tracks();
    ASSERT_EQ(tracks_after.size(), 1);
    EXPECT_EQ(tracks_after.front().misses(), 1);
}

TEST_F(LineTrackManagerTests, rho_gate_filters_distant_measurement) {
    auto cfg = make_default_config();
    cfg.default_config.max_rho_error = 0.5;  // tight rho gate
    LineTrackManager mgr(cfg);

    std::vector<LineMeasurement> z1 = {make_measurement(1.0, 0.0)};
    mgr.step(z1, 0.1);
    ASSERT_EQ(mgr.get_tracks().size(), 1);

    std::vector<LineMeasurement> z2 = {make_measurement(5.0, 0.0)};
    mgr.step(z2, 0.1);

    EXPECT_GE(mgr.get_tracks().size(), 2u);
}

TEST_F(LineTrackManagerTests, phi_gate_filters_rotated_measurement) {
    auto cfg = make_default_config();
    cfg.default_config.max_phi_error = 0.2;  // tight angle gate
    LineTrackManager mgr(cfg);

    std::vector<LineMeasurement> z1 = {make_measurement(1.0, 0.0)};
    mgr.step(z1, 0.1);
    ASSERT_EQ(mgr.get_tracks().size(), 1);

    std::vector<LineMeasurement> z2 = {make_measurement(1.0, 1.0)};
    mgr.step(z2, 0.1);

    EXPECT_GE(mgr.get_tracks().size(), 2u);
}

TEST_F(LineTrackManagerTests, sign_alignment_handles_flipped_normal) {
    auto cfg = make_default_config();
    LineTrackManager mgr(cfg);

    std::vector<LineMeasurement> z1 = {make_measurement(1.0, 0.0)};
    mgr.step(z1, 0.1);

    ASSERT_EQ(mgr.get_tracks().size(), 1);
    int hits1 = mgr.get_tracks().front().hits();

    // Equivalent line with flipped sign: (rho=-1.0, n=[-1, 0])
    std::vector<LineMeasurement> z2 = {make_measurement(-1.0, M_PI)};
    mgr.step(z2, 0.1);

    // Should still be one track (the flipped measurement was associated)
    ASSERT_EQ(mgr.get_tracks().size(), 1);
    EXPECT_GT(mgr.get_tracks().front().hits(), hits1);
}

TEST_F(LineTrackManagerTests, track_confirmed_after_n_hits) {
    auto cfg = make_default_config();
    cfg.nm.confirm_n = 3;
    cfg.nm.confirm_m = 5;
    LineTrackManager mgr(cfg);

    for (int i = 0; i < 5; ++i) {
        std::vector<LineMeasurement> z = {make_measurement(1.0, 0.0)};
        mgr.step(z, 0.1);
    }

    ASSERT_EQ(mgr.get_tracks().size(), 1);
    EXPECT_TRUE(mgr.get_tracks().front().confirmed);
}

TEST_F(LineTrackManagerTests, track_deleted_after_n_misses) {
    auto cfg = make_default_config();
    cfg.nm.confirm_n = 1;
    cfg.nm.confirm_m = 1;
    cfg.nm.delete_n = 3;
    cfg.nm.delete_m = 3;
    LineTrackManager mgr(cfg);

    std::vector<LineMeasurement> z = {make_measurement(1.0, 0.0)};
    mgr.step(z, 0.1);
    ASSERT_EQ(mgr.get_tracks().size(), 1);

    for (int i = 0; i < 3; ++i) {
        z = {};
        mgr.step(z, 0.1);
    }

    EXPECT_EQ(mgr.get_tracks().size(), 0);
}

TEST_F(LineTrackManagerTests, multiple_tracks_maintained_independently) {
    auto cfg = make_default_config();
    LineTrackManager mgr(cfg);

    std::vector<LineMeasurement> z = {
        make_measurement(1.0, 0.0),
        make_measurement(5.0, M_PI / 2),
    };
    mgr.step(z, 0.1);
    ASSERT_EQ(mgr.get_tracks().size(), 2);

    // Observe only the first track
    z = {make_measurement(1.0, 0.0)};
    mgr.step(z, 0.1);

    ASSERT_GE(mgr.get_tracks().size(), 1u);

    bool found_first = false;
    for (const auto& t : mgr.get_tracks()) {
        if (std::abs(t.nominal.rho - 1.0) < 0.5 &&
            std::abs(t.nominal.n.x() - 1.0) < 0.5) {
            found_first = true;
            EXPECT_EQ(t.hits(), 2);
        }
    }
    EXPECT_TRUE(found_first);
}

TEST_F(LineTrackManagerTests, from_segment_creates_valid_measurement) {
    LineSegment2D seg;
    seg.p0 = {0.0, 0.0};
    seg.p1 = {1.0, 0.0};

    auto meas = LineMeasurement::from_segment(seg);

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

    EXPECT_NEAR(meas.n.norm(), 1.0, 1e-12);
    EXPECT_NEAR(meas.rho, 0.0, 1e-12);
}

TEST_F(LineTrackManagerTests, from_segment_offset_line) {
    LineSegment2D seg;
    seg.p0 = {0.0, 2.0};
    seg.p1 = {1.0, 2.0};

    auto meas = LineMeasurement::from_segment(seg);

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

    std::vector<LineMeasurement> z = {make_measurement(-2.0, M_PI)};
    mgr.step(z, 0.1);

    ASSERT_EQ(mgr.get_tracks().size(), 1);

    auto canonical = mgr.get_tracks().front().to_canonical_line();
    EXPECT_GE(canonical.rho, 0.0);
    EXPECT_GE(canonical.theta, 0.0);
    EXPECT_LT(canonical.theta, 2.0 * M_PI);
}

}  // namespace vortex::line_filtering
