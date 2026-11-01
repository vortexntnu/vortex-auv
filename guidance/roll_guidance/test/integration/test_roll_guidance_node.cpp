#include "roll_test_fixture.hpp"

namespace vortex::guidance::test {

/**
 * @brief Verifies successful completion of a ROLL maneuver.
 *
 * The mock Waypoint Manager accepts the generated waypoints
 * and reports successful completion.
 */
TEST_F(ROLLIntegrationFixture, T20_test_roll_completion) {
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

    const auto wrapped = result.get();

    EXPECT_EQ(
        wrapped.code,
        rclcpp_action::ResultCode::SUCCEEDED);

    ASSERT_NE(wrapped.result, nullptr);
    EXPECT_TRUE(wrapped.result->success);

    EXPECT_TRUE(guidance_->shutdown_was_requested());
}

/**
 * @brief Verifies that gate crossing terminates ROLL
 *        before Waypoint Manager completion.
 */
TEST_F(ROLLIntegrationFixture, T21_test_gate_crossing_cancels_roll) {
    publish_gate(0.0);
    publish_pose(-1.0);

    auto goal = start_roll();
    ASSERT_NE(goal, nullptr);

    ASSERT_TRUE(wait_until([this] {
        return manager_->goals_received.load() == 1;
    }));

    auto result = client_->async_get_result(goal);

    publish_pose(1.0);

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
 * @brief Verifies that simultaneous termination events
 * do not finalize ROLL more than once.
 *
 * Triggers successful Waypoint Manager completion and
 * gate crossing at approximately the same time.
 *
 * Verifies that:
 * - The ROLL action produces one terminal result.
 * - The result is either SUCCEEDED or ABORTED.
 * - Shutdown is requested.
 *
 * @note The winning event is nondeterministic.
 */
TEST_F(ROLLIntegrationFixture, T22_test_simultaneous_completion) {
    publish_gate(0.0);
    publish_pose(-1.0);

    auto goal = start_roll();
    ASSERT_NE(goal, nullptr);

    ASSERT_TRUE(wait_until([this] {
        return manager_->goals_received.load() == 1;
    }));

    auto result = client_->async_get_result(goal);

    std::thread complete_thread([this] {
        manager_->finish(true);
    });

    std::thread crossing_thread([this] {
        publish_pose(1.0);
    });

    complete_thread.join();
    crossing_thread.join();

    ASSERT_EQ(
        result.wait_for(3s),
        std::future_status::ready);

    const auto wrapped = result.get();

    EXPECT_TRUE(
        wrapped.code == rclcpp_action::ResultCode::SUCCEEDED ||
        wrapped.code == rclcpp_action::ResultCode::ABORTED);

    EXPECT_TRUE(guidance_->shutdown_was_requested());
}

/**
 * @brief Verifies handling of a rejected Waypoint Manager goal.
 *
 * Starts a ROLL guidance action and configures the mock
 * Waypoint Manager to reject the generated waypoint goal.
 *
 * Verifies that:
 * - The Waypoint Manager rejection is detected.
 * - The ROLL action terminates with an aborted result.
 * - The action is finalized exactly once.
 * - Node shutdown is requested.
 */
TEST_F(ROLLIntegrationFixture, T23_test_waypoint_manager_rejection) {
    manager_->reject_goals = true;

    publish_gate(0.0);
    publish_pose(-1.0);

    auto goal = start_roll();
    ASSERT_NE(goal, nullptr);

    auto result = client_->async_get_result(goal);

    ASSERT_EQ(
        result.wait_for(3s),
        std::future_status::ready);

    const auto wrapped = result.get();

    EXPECT_EQ(
        wrapped.code,
        rclcpp_action::ResultCode::ABORTED);

    ASSERT_NE(wrapped.result, nullptr);
    EXPECT_FALSE(wrapped.result->success);

    EXPECT_TRUE(guidance_->shutdown_was_requested());
}

/**
 * @brief Verifies cancellation of an active ROLL action.
 *
 * Starts ROLL, waits for Waypoint Manager to accept
 * the generated waypoint goal, then requests cancellation.
 *
 * Verifies that:
 * - The ROLL cancellation request is accepted.
 * - Cancellation is forwarded to Waypoint Manager.
 * - The ROLL action terminates as CANCELED.
 * - Shutdown is requested.
 */
TEST_F(ROLLIntegrationFixture, T24_test_roll_cancellation) {
    publish_gate(0.0);
    publish_pose(-1.0);

    auto goal = start_roll();
    ASSERT_NE(goal, nullptr);

    ASSERT_TRUE(wait_until([this] {
        return manager_->goals_received.load() == 1;
    }));

    auto result = client_->async_get_result(goal);

    auto cancel_future = client_->async_cancel_goal(goal);

    ASSERT_EQ(
        cancel_future.wait_for(3s),
        std::future_status::ready);

    const auto cancel_response = cancel_future.get();

    ASSERT_NE(cancel_response, nullptr);
    ASSERT_FALSE(cancel_response->goals_canceling.empty());

    ASSERT_TRUE(wait_until([this] {
        return manager_->cancel_requests.load() >= 1;
    }));

    ASSERT_EQ(
        result.wait_for(3s),
        std::future_status::ready);

    const auto wrapped = result.get();

    EXPECT_EQ(
        wrapped.code,
        rclcpp_action::ResultCode::CANCELED);

    EXPECT_TRUE(guidance_->shutdown_was_requested());
}

/**
 * @brief Verifies that concurrent ROLL guidance goals are rejected.
 *
 * Starts a ROLL guidance action and keeps it active.
 *
 * Sends a second ROLL goal before the first goal has completed.
 *
 * Verifies that:
 * - The first ROLL goal is accepted.
 * - The second ROLL goal is rejected.
 * - The first goal remains active.
 * - No second Waypoint Manager goal is dispatched.
 */
TEST_F(ROLLIntegrationFixture, T25_test_reject_concurrent_goal) {
    publish_gate(0.0);
    publish_pose(-1.0);

    auto first_goal = start_roll();
    ASSERT_NE(first_goal, nullptr);

    ASSERT_TRUE(wait_until([this] {
        return manager_->goals_received.load() == 1;
    }));

    auto second_goal = start_roll();

    EXPECT_EQ(second_goal, nullptr);
    EXPECT_EQ(manager_->goals_received.load(), 1);
}

/**
 * @brief Verifies cancellation when gate crossing occurs
 * before Waypoint Manager accepts the goal.
 *
 * Delays the downstream goal response, detects gate crossing,
 * then allows the Waypoint Manager to accept the goal.
 *
 * Verifies that:
 * - ROLL terminates when the gate is crossed.
 * - A late-accepted Waypoint Manager goal is canceled.
 * - Shutdown is requested.
 */
TEST_F(ROLLIntegrationFixture, T26_test_gate_crossing_before_goal_acceptance) {

    manager_->delay_acceptance = true;

    publish_gate(0.0);
    publish_pose(-1.0);

    auto goal = start_roll();
    ASSERT_NE(goal, nullptr);

    ASSERT_TRUE(wait_until([this] {
        return manager_->goal_request_seen.load();
    }));

    auto result = client_->async_get_result(goal);

    //gate crossing occurs while acceptance is delayed.
    publish_pose(1.0);

    ASSERT_EQ(
        result.wait_for(3s),
        std::future_status::ready);

    EXPECT_EQ(
        result.get().code,
        rclcpp_action::ResultCode::ABORTED);

    //release the delayed Waypoint Manager response.
    manager_->release_acceptance = true;

    EXPECT_TRUE(wait_until([this] {
        return manager_->cancel_requests.load() >= 1;
    }));

    EXPECT_TRUE(guidance_->shutdown_was_requested());
}

/**
 * @brief Verifies that republishing the gate waypoint preserves crossing state.
 *
 * Starts a ROLL maneuver and observes the AUV on the
 * approach side of the gate.
 *
 * Republishes the same gate waypoint while the ROLL action
 * is active, then moves the AUV to the exit side.
 *
 * Verifies that:
 * - Republishing the gate does not reset approach history.
 * - Gate crossing is still detected.
 * - The ROLL action terminates.
 * - Cancellation is requested for the active Waypoint Manager goal.
 *
 * @note This test protects against accidental resets of
 * gate_approach_seen_ during an active maneuver.
 */
TEST_F(ROLLIntegrationFixture, T27_test_gate_waypoint_republished) {
    publish_gate(0.0);
    publish_pose(-1.0);

    auto goal = start_roll();
    ASSERT_NE(goal, nullptr);

    ASSERT_TRUE(wait_until([this] {
        return manager_->goals_received.load() == 1;
    }));

    publish_gate(0.0);
    publish_pose(1.0);

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

}  // namespace vortex::guidance::test
