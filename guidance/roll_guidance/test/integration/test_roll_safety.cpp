
#include "roll_test_fixture.hpp"

#include <atomic>
#include <chrono>
#include <cmath>
#include <thread>

namespace vortex::guidance::test {

/**
 * @brief Verifies that Waypoint Manager success alone
 *        cannot complete a ROLL action.
 *
 * The measured orientation remains unchanged while the mock
 * Waypoint Manager reports success.
 *
 * @test ROLL terminates as ABORTED.
 */
TEST_F(ROLLIntegrationFixture, T35_manager_success_without_measured_roll) {

    publish_killswitch(false);
    publish_gate(0.0);
    publish_pose(-1.0);

    auto goal = start_roll();
    ASSERT_NE(goal, nullptr);

    ASSERT_TRUE(wait_until([this] {
        return manager_->goals_received.load() == 1;
    }));

    auto result = client_->async_get_result(goal);

    ASSERT_TRUE(manager_->finish(true));

    ASSERT_EQ(
        result.wait_for(3s),
        std::future_status::ready);

    EXPECT_EQ(
        result.get().code,
        rclcpp_action::ResultCode::ABORTED);
}

/**
 * @brief Verifies successful completion of a measured roll.
 *
 * Publishes a full 360-degree sequence of measured orientations
 * before the Waypoint Manager reports success.
 *
 * @test Accumulated roll and final orientation are accepted.
 * @test ROLL terminates as SUCCEEDED.
 */
TEST_F(ROLLIntegrationFixture, T36_measured_roll_success) {

    publish_killswitch(false);
    publish_gate(0.0);
    publish_pose(-1.0);

    auto goal = start_roll();
    ASSERT_NE(goal, nullptr);

    ASSERT_TRUE(wait_until([this] {
        return manager_->goals_received.load() == 1;
    }));

    auto result = client_->async_get_result(goal);

    publish_measured_roll(2.0 * M_PI);

    ASSERT_TRUE(manager_->finish(true));

    ASSERT_EQ(
        result.wait_for(3s),
        std::future_status::ready);

    const auto wrapped = result.get();

    EXPECT_EQ(
        wrapped.code,
        rclcpp_action::ResultCode::SUCCEEDED);

    ASSERT_NE(wrapped.result, nullptr);
    EXPECT_TRUE(wrapped.result->success);
}

/**
 * @brief Verifies that a completed roll with incorrect
 * final orientation does not succeed.
 *
 * The vehicle completes a full roll, then deviates by
 * ten degrees around the yaw axis.
 *
 * @test Final orientation verification rejects the maneuver.
 */
TEST_F(ROLLIntegrationFixture, T37_incorrect_final_orientation) {

    publish_killswitch(false);
    publish_gate(0.0);
    publish_pose(-1.0);

    auto goal = start_roll();
    ASSERT_NE(goal, nullptr);

    ASSERT_TRUE(wait_until([this] {
        return manager_->goals_received.load() == 1;
    }));

    auto result = client_->async_get_result(goal);

    publish_measured_roll(2.0 * M_PI);

    const Eigen::Quaterniond yaw_offset(
        Eigen::AngleAxisd(
            10.0 * M_PI / 180.0,
            Eigen::Vector3d::UnitZ()));

    publish_oriented_pose(-1.0, yaw_offset);

    std::this_thread::sleep_for(100ms);

    ASSERT_TRUE(manager_->finish(true));

    ASSERT_EQ(
        result.wait_for(3s),
        std::future_status::ready);

    EXPECT_EQ(
        result.get().code,
        rclcpp_action::ResultCode::ABORTED);
}

/**
 * @brief Verifies stale-pose detection during a maneuver.
 *
 * Stops publishing poses after the Waypoint Manager accepts
 * the generated roll waypoints.
 *
 * @test ROLL aborts after the pose freshness timeout.
 * @test Downstream cancellation is requested.
 */
class ROLLStalePoseTests : public ROLLIntegrationFixture {
protected:
    /**
     * @brief Configures a short pose freshness timeout.
     * @return Timeout in seconds.
     */
    double pose_timeout() const override {
        return 0.35;
    }
};

TEST_F(ROLLStalePoseTests, T38_stale_pose_during_maneuver) {

    publish_killswitch(false);
    publish_gate(0.0);
    publish_pose(-1.0);

    auto goal = start_roll();
    ASSERT_NE(goal, nullptr);

    ASSERT_TRUE(wait_until([this] {
        return manager_->goals_received.load() == 1;
    }));

    auto result = client_->async_get_result(goal);

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
 * @brief Verifies that missing pose measurements cause
 * an action timeout.
 *
 * No valid pose is published after accepting the ROLL goal.
 *
 * @test ROLL aborts instead of waiting indefinitely.
 */
TEST_F(ROLLStalePoseTests, T39_no_valid_pose) {

    publish_killswitch(false);
    publish_gate(0.0);

    auto goal = start_roll();
    ASSERT_NE(goal, nullptr);

    auto result = client_->async_get_result(goal);

    ASSERT_EQ(
        result.wait_for(3s),
        std::future_status::ready);

    EXPECT_EQ(
        result.get().code,
        rclcpp_action::ResultCode::ABORTED);

    EXPECT_EQ(manager_->goals_received.load(), 0);
}

/**
 * @brief Configures a short trigger timeout.
 */
class ROLLTriggerTimeoutTests : public ROLLIntegrationFixture {
protected:
    /**
     * @brief Returns the trigger timeout for this test fixture.
     * @return Timeout in seconds.
     */
    double trigger_timeout() const override {
        return 0.35;
    }
};

/**
 * @brief Verifies that the action aborts when the AUV
 * never reaches the gate trigger distance.
 *
 * The gate is placed far from the measured AUV position.
 *
 * @test No Waypoint Manager goal is sent.
 * @test ROLL aborts after the trigger timeout.
 */
TEST_F( ROLLTriggerTimeoutTests, T40_gate_trigger_timeout) {

    publish_killswitch(false);
    publish_gate(100.0);
    publish_pose(-1.0);

    auto goal = start_roll();
    ASSERT_NE(goal, nullptr);

    auto result = client_->async_get_result(goal);

    ASSERT_EQ(
        result.wait_for(3s),
        std::future_status::ready);

    EXPECT_EQ(
        result.get().code,
        rclcpp_action::ResultCode::ABORTED);

    EXPECT_EQ(manager_->goals_received.load(), 0);
}

/**
 * @brief Configures a short maneuver timeout.
 */
class ROLLManeuverTimeoutTests : public ROLLIntegrationFixture {
protected:
    /**
     * @brief Returns the maneuver timeout for this test fixture.
     * @return Timeout in seconds.
     */
    double maneuver_timeout() const override {
        return 0.35;
    }
};

/**
 * @brief Verifies that an unfinished maneuver times out.
 *
 * The mock Waypoint Manager accepts the goal but never reports
 * successful completion.
 *
 * @test ROLL aborts after the maneuver timeout.
 * @test The manager receives a cancellation request.
 */
TEST_F(ROLLManeuverTimeoutTests, T41_maneuver_timeout) {

    publish_killswitch(false);
    publish_gate(0.0);
    publish_pose(-1.0);

    auto goal = start_roll();
    ASSERT_NE(goal, nullptr);

    ASSERT_TRUE(wait_until([this] {
        return manager_->goals_received.load() == 1;
    }));

    auto result = client_->async_get_result(goal);

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
 * @brief Verifies killswitch-triggered cancellation
 * during an active ROLL maneuver.
 *
 * Starts the maneuver with the killswitch released and
 * engages it after the manager accepts the goal.
 *
 * @test ROLL aborts.
 * @test Waypoint Manager cancellation is requested.
 */
TEST_F(ROLLIntegrationFixture, T42_killswitch_engaged_during_roll) {

    publish_killswitch(false);
    publish_gate(0.0);
    publish_pose(-1.0);

    auto goal = start_roll();
    ASSERT_NE(goal, nullptr);

    ASSERT_TRUE(wait_until([this] {
        return manager_->goals_received.load() == 1;
    }));

    auto result = client_->async_get_result(goal);

    publish_killswitch(true);

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
 * @brief Verifies that an engaged killswitch prevents
 * the ROLL maneuver from starting.
 *
 * Publishes the engaged killswitch state before the goal.
 *
 * @test No Waypoint Manager goal is dispatched.
 * @test ROLL eventually aborts.
 */
TEST_F(ROLLTriggerTimeoutTests, T43_killswitch_prevents_start) {

    publish_killswitch(true);
    publish_gate(0.0);
    publish_pose(-1.0);

    auto goal = start_roll();
    ASSERT_NE(goal, nullptr);

    auto result = client_->async_get_result(goal);

    ASSERT_EQ(
        result.wait_for(3s),
        std::future_status::ready);

    EXPECT_EQ(
        result.get().code,
        rclcpp_action::ResultCode::ABORTED);

    EXPECT_EQ(manager_->goals_received.load(), 0);
}


/**
 * @brief Verifies cancellation before the Waypoint
 * Manager accepts the downstream goal.
 *
 * The mock manager delays acceptance. The user cancels the
 * ROLL action while the downstream goal response is pending.
 * The manager then accepts the goal.
 *
 * @test The upstream ROLL action is canceled.
 * @test The late-accepted downstream goal is canceled.
 */
TEST_F(ROLLIntegrationFixture, T44_cancel_before_manager_acceptance) {

    manager_->delay_acceptance = true;

    publish_killswitch(false);
    publish_gate(0.0);
    publish_pose(-1.0);

    auto goal = start_roll();
    ASSERT_NE(goal, nullptr);

    ASSERT_TRUE(wait_until([this] {
        return manager_->goal_request_seen.load();
    }));

    auto result = client_->async_get_result(goal);

    auto cancel_future = client_->async_cancel_goal(goal);

    ASSERT_EQ(
        cancel_future.wait_for(3s),
        std::future_status::ready);

    const auto cancel_response = cancel_future.get();

    ASSERT_NE(cancel_response, nullptr);
    ASSERT_FALSE(cancel_response->goals_canceling.empty());

    //allow the downstream manager to accept the goal.
    manager_->release_acceptance = true;

    ASSERT_EQ(
        result.wait_for(3s),
        std::future_status::ready);

    EXPECT_EQ(
        result.get().code,
        rclcpp_action::ResultCode::CANCELED);

    EXPECT_TRUE(wait_until([this] {
        return manager_->cancel_requests.load() >= 1;
    }));
}

/**
 * @brief Verifies that simultaneous cancellation and
 * timeout do not produce duplicate action finalization.
 *
 * Requests cancellation near the maneuver timeout boundary.
 *
 * @test The ROLL action produces one terminal result.
 * @test The terminal state is CANCELED or ABORTED,
 * depending on which event wins.
 */
TEST_F(ROLLManeuverTimeoutTests, T45_cancellation_timeout_race) {

    publish_killswitch(false);
    publish_gate(0.0);
    publish_pose(-1.0);

    auto goal = start_roll();
    ASSERT_NE(goal, nullptr);

    ASSERT_TRUE(wait_until([this] {
        return manager_->goals_received.load() == 1;
    }));

    auto result = client_->async_get_result(goal);

    std::thread cancel_thread([this, goal] {
        std::this_thread::sleep_for(300ms);
        client_->async_cancel_goal(goal);
    });

    const auto status = result.wait_for(3s);

    cancel_thread.join();

    ASSERT_EQ(status, std::future_status::ready);

    const auto wrapped = result.get();

    EXPECT_TRUE(
        wrapped.code == rclcpp_action::ResultCode::CANCELED ||
        wrapped.code == rclcpp_action::ResultCode::ABORTED);

    EXPECT_TRUE(wait_until([this] {
        return manager_->cancel_requests.load() >= 1;
    }));
}


/**
 * @brief Verifies rejection of out-of-order pose timestamps.
 *
 * Publishes a valid pose with a newer timestamp, followed
 * by a pose whose timestamp is older.
 *
 * @test The older pose must not replace the latest valid pose.
 *
 * @note Requires timestamp-order validation in pose_callback().
 * Also requires a test-observable accepted-pose counter
 * or last-accepted timestamp accessor.
 */
TEST_F(ROLLIntegrationFixture, T46_out_of_order_pose_timestamp) {

    publish_killswitch(false);
    publish_gate(0.0);

    // TODO: Add a publish_pose_at_time() fixture helper.
    // TODO: Publish a pose with timestamp T2.
    // TODO: Wait until T2 is accepted.
    // TODO: Publish another pose with timestamp T1 < T2.
    // TODO: Assert T1 was rejected.
    // TODO: Assert the accepted-pose counter did not advance.

    GTEST_SKIP() <<
        "Requires pose timestamp validation and "
        "test-observable accepted-pose state";
}


}  // namespace vortex::guidance::test
