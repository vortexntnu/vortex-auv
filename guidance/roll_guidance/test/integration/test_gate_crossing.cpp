#include "roll_test_fixture.hpp"

namespace vortex::guidance::test {

/**
 * @brief Verifies that approaching the gate without crossing
 *        does not terminate ROLL guidance.
 */
TEST_F(ROLLIntegrationFixture, T13_test_approach_only) {
    publish_gate(0.0);
    publish_pose(-2.0);

    auto goal = start_roll();
    ASSERT_NE(goal, nullptr);

    ASSERT_TRUE(wait_until([this] {
        return manager_->goals_received.load() == 1;
    }));

    publish_pose(-1.0);
    publish_pose(-0.2);

    auto result = client_->async_get_result(goal);

    EXPECT_EQ(
        result.wait_for(300ms),
        std::future_status::timeout);

    EXPECT_EQ(manager_->cancel_requests.load(), 0);
}

/**
 * @brief Verifies that forward crossing terminates ROLL
 * and requests Waypoint Manager cancellation.
 */
TEST_F(ROLLIntegrationFixture, T14_test_gate_crossing) {
    publish_gate(0.0);
    publish_pose(-2.0);

    auto goal = start_roll();
    ASSERT_NE(goal, nullptr);

    ASSERT_TRUE(wait_until([this] {
        return manager_->goals_received.load() == 1;
    }));

    publish_pose(-1.0);
    publish_pose(1.0);

    auto result = client_->async_get_result(goal);

    ASSERT_EQ(
        result.wait_for(3s),
        std::future_status::ready);

    EXPECT_EQ(
        result.get().code,
        rclcpp_action::ResultCode::ABORTED);

    EXPECT_TRUE(wait_until([this] {
        return manager_->cancel_requests.load() > 0;
    }));

    EXPECT_TRUE(guidance_->shutdown_was_requested());
}

 
/**
 * @brief Verifies that starting beyond the gate does not count as crossing.
 *
 * Configures a gate at the origin with its normal pointing
 * along the positive X-axis.
 *
 * Simulates an AUV that is already on the exit side of the gate
 * without previously being observed on the approach side.
 *
 * Verifies that:
 * - The initial position does not trigger crossing.
 * - Remaining on the exit side does not trigger crossing.
 *
 * @note Gate crossing requires prior observation on the
 * approach side.
 */
TEST_F(ROLLIntegrationFixture, T15_test_start_on_exit_side) {
    publish_gate(0.0);
    publish_pose(1.0);

    auto goal = start_roll();
    ASSERT_NE(goal, nullptr);

    ASSERT_TRUE(wait_until([this] {
        return manager_->goals_received.load() == 1;
    }));

    publish_pose(2.0);

    auto result = client_->async_get_result(goal);

    EXPECT_EQ(
        result.wait_for(300ms),
        std::future_status::timeout);

    EXPECT_EQ(manager_->cancel_requests.load(), 0);
}

/**
 * @brief Verifies that crossing the gate in the wrong direction is ignored.
 *
 * Configures a gate at the origin with its normal pointing
 * along the positive X-axis.
 *
 * Simulates an AUV moving from the positive X-axis to
 * the negative X-axis.
 *
 * Verifies that:
 * - Movement in the reverse direction is not considered
 *   a valid gate crossing.
 * - The gate-crossing state remains false.
 */
TEST_F(ROLLIntegrationFixture, T16_test_reverse_gate_crossing) {
    publish_gate(0.0);
    publish_pose(1.0);

    auto goal = start_roll();
    ASSERT_NE(goal, nullptr);

    ASSERT_TRUE(wait_until([this] {
        return manager_->goals_received.load() == 1;
    }));

    publish_pose(-1.0);

    auto result = client_->async_get_result(goal);

    EXPECT_EQ(
        result.wait_for(300ms),
        std::future_status::timeout);

    EXPECT_EQ(manager_->cancel_requests.load(), 0);
}

/**
 * @brief Verifies that crossing requires movement
 * beyond the 0.05-meter tolerance.
 */
TEST_F(ROLLIntegrationFixture, T17_test_gate_crossing_tolerance) {
    publish_gate(0.0);
    publish_pose(-0.10);

    auto goal = start_roll();
    ASSERT_NE(goal, nullptr);

    ASSERT_TRUE(wait_until([this] {
        return manager_->goals_received.load() == 1;
    }));

    publish_pose(0.04);

    auto result = client_->async_get_result(goal);

    EXPECT_EQ(
        result.wait_for(300ms),
        std::future_status::timeout);

    publish_pose(0.10);

    ASSERT_EQ(
        result.wait_for(3s),
        std::future_status::ready);

    EXPECT_EQ(
        result.get().code,
        rclcpp_action::ResultCode::ABORTED);
}

/**
 * @brief Configures the gate normal along the positive Y-axis.
 */
class ROLLYAxisGateTests : public ROLLIntegrationFixture {
   protected:
    std::vector<double> gate_normal() const override {
        return {0.0, 1.0, 0.0};
    }
};

/**
 * @brief Verifies gate crossing along the Y-axis.
 *
 * Verifies that crossing from negative Y to positive Y
 * terminates ROLL and requests Waypoint Manager cancellation.
 */
TEST_F(ROLLYAxisGateTests, T18_test_y_axis_gate_crossing) {
    publish_gate(0.0, 0.0, 0.0);
    publish_pose(0.0, -1.0, 0.0);

    auto goal = start_roll();
    ASSERT_NE(goal, nullptr);

    ASSERT_TRUE(wait_until([this] {
        return manager_->goals_received.load() == 1;
    }));

    auto result = client_->async_get_result(goal);

    publish_pose(0.0, 1.0, 0.0);

    ASSERT_EQ(
        result.wait_for(3s),
        std::future_status::ready);

    EXPECT_EQ(
        result.get().code,
        rclcpp_action::ResultCode::ABORTED);

    EXPECT_TRUE(wait_until([this] {
        return manager_->cancel_requests.load() >= 1;
    }));
}

/**
 * @brief Verifies that gate-crossing detection remains latched.
 *
 * Simulates an AUV crossing the gate successfully and
 * subsequently receiving additional pose updates.
 *
 * Verifies that:
 * - The first valid crossing is detected.
 * - Subsequent pose updates do not reset the crossing state.
 * - Once detected, the crossing state remains true.
 */
TEST_F(ROLLIntegrationFixture, T19_test_gate_crossing_latched) {
    publish_gate(0.0);
    publish_pose(-1.0);

    auto goal = start_roll();
    ASSERT_NE(goal, nullptr);

    ASSERT_TRUE(wait_until([this] {
        return manager_->goals_received.load() == 1;
    }));

    publish_pose(1.0);

    auto result = client_->async_get_result(goal);

    ASSERT_EQ(
        result.wait_for(3s),
        std::future_status::ready);

    EXPECT_EQ(
        result.get().code,
        rclcpp_action::ResultCode::ABORTED);

    publish_pose(-1.0);
    publish_pose(2.0);

    EXPECT_TRUE(guidance_->shutdown_was_requested());
}

}  // namespace vortex::guidance::test
