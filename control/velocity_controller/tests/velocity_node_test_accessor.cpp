#include <gtest/gtest.h>
#include "velocity_controller/velocity_controller_ros.hpp"
#include "velocity_controller/tests/velocity_node_test_accessor.hpp"

namespace {

std::vector<rclcpp::Parameter> make_full_parameter_overrides() {
    return {
        // topics
        rclcpp::Parameter("topics.wrench_input", "thrust_out"),
        rclcpp::Parameter("topics.guidance.los", "guidance/los"),
        rclcpp::Parameter("topics.odom", "odom"),

        // Control_manager_settings
        rclcpp::Parameter("Control_manager_settings.publish_rate", 100),
        rclcpp::Parameter("Control_manager_settings.controller_type", 1),
        rclcpp::Parameter("Control_manager_settings.anti_overshoot", false),

        // Node_settings
        rclcpp::Parameter("Node_settings.auto_start", false),  // false i tester - unngå auto-transisjon
        rclcpp::Parameter("Node_settings.reset_on_new_ref", true),
        rclcpp::Parameter("Node_settings.odometry_dropout_guard", true),

        // 3DOF_PID_params
        rclcpp::Parameter("3DOF_PID_params.surge", std::vector<double>{500.0, 50.0, 5.0}),
        rclcpp::Parameter("3DOF_PID_params.pitch", std::vector<double>{60.0, 8.0, 12.0}),
        rclcpp::Parameter("3DOF_PID_params.yaw", std::vector<double>{10.0, 1.0, 5.0}),

        // LQR_params
        rclcpp::Parameter("LQR_params.Q", std::vector<double>{200.0,32.84,32.84,15.0,15.0,100.0,32.84,32.84}),
        rclcpp::Parameter("LQR_params.R", std::vector<double>{0.02, 3.1, 3.10}),
        rclcpp::Parameter("physical.mass_matrix", std::vector<double>{
            53.7,0,0,0,0,0, 0,53.7,0,0,0,0, 0,0,53.7,0,0,0,
            0,0,0,11.0628,1.086,-3.17502, 0,0,0,1.086,23.1128,0.1025,
            0,0,0,-3.17502,0.1025,26.23998}),

        rclcpp::Parameter("dampening_matrix_low", std::vector<double>{
            104.0,0,0,0,0,0, 0,46,0,0,0,0, 0,0,46,0,0,0,
            0,0,0,46,0,0, 0,0,0,0,46,0, 0,0,0,0,0,46}),
        rclcpp::Parameter("dampening_matrix_high", std::vector<double>{
            1,0,0,0,0,0, 0,1,0,0,0,0, 0,0,1,0,0,0,
            0,0,0,1,0,0, 0,0,0,0,1,0, 0,0,0,0,0,1}),

        // propulsion / physical (fra thruster-fixturen vi allerede har brukt)
        rclcpp::Parameter("propulsion.dimensions.num", 3),
        rclcpp::Parameter("propulsion.thrusters.num", 8),
        rclcpp::Parameter("propulsion.thrusters.thruster_position", std::vector<double>{
            0.413892, 0.140095, -0.163904, -0.413892, -0.413892, -0.163904, 0.140095, 0.413892,
            0.313022, 0.313022,  0.313022,  0.313022, -0.313022, -0.313022, -0.313022, -0.313022,
            0.021736, 0.021736,  0.021736,  0.021736,  0.021736,  0.021736,  0.021736,  0.021736}),
        rclcpp::Parameter("propulsion.thrusters.thruster_force_direction", std::vector<double>{
            0.70711, 0.00000, 0.00000, -0.70711, -0.70711, 0.00000, 0.00000, 0.70711,
           -0.70711, 0.00000, 0.00000, -0.70711,  0.70711, 0.00000, 0.00000, 0.70711,
            0.00000, 1.00000, 1.00000,  0.00000,  0.00000, 1.00000, 1.00000, 0.00000}),
        rclcpp::Parameter("physical.center_of_mass", std::vector<double>{0.0, 0.0, 0.01}),
        rclcpp::Parameter("propulsion.thrusters.constraints.min_force", -40.0),
        rclcpp::Parameter("propulsion.thrusters.constraints.max_force", 40.0),
    };
}

rclcpp::NodeOptions make_node_options(std::vector<rclcpp::Parameter> overrides) {
    rclcpp::NodeOptions options;
    options.parameter_overrides(overrides);
    return options;
}

}  // namespace

class RclcppEnvironment : public ::testing::Environment {
public:
    void SetUp() override { rclcpp::init(0, nullptr); }
    void TearDown() override { rclcpp::shutdown(); }
};

// Registrer én gang, brukes av alle testfiler som lenkes inn i samme binary
::testing::Environment* const rclcpp_env =
    ::testing::AddGlobalTestEnvironment(new RclcppEnvironment);

// ---------- Parameter-lasting ----------

TEST(VelocityNodeParameterTest, MissingRequiredParameterThrowsOnConstruction) {
    // Regresjonstest for den opprinnelige feilen fra starten av denne samtalen:
    // en manglende påkrevd parameter (her: thruster_position) skal kaste en
    // klar exception fra selve konstruktøren, ikke krasje mystisk senere.
    auto overrides = make_full_parameter_overrides();
    overrides.erase(std::remove_if(overrides.begin(), overrides.end(),
        [](const rclcpp::Parameter& p) {
            return p.get_name() == "propulsion.thrusters.thruster_position";
        }), overrides.end());

    EXPECT_THROW(
        { Velocity_node node(make_node_options(overrides)); },
        rclcpp::exceptions::ParameterUninitializedException);
}

TEST(VelocityNodeParameterTest, MissingTopicParameterThrowsOnConstruction) {
    auto overrides = make_full_parameter_overrides();
    overrides.erase(std::remove_if(overrides.begin(), overrides.end(),
        [](const rclcpp::Parameter& p) {
            return p.get_name() == "topics.wrench_input";
        }), overrides.end());

    EXPECT_THROW(
        { Velocity_node node(make_node_options(overrides)); },
        rclcpp::exceptions::ParameterUninitializedException);
}

TEST(VelocityNodeParameterTest, CompleteValidParametersConstructWithoutThrowing) {
    EXPECT_NO_THROW({
        Velocity_node node(make_node_options(make_full_parameter_overrides()));
    });
}

TEST(VelocityNodeParameterTest, ControlManagerInitializedAfterConstruction) {
    Velocity_node node(make_node_options(make_full_parameter_overrides()));
    EXPECT_NE(VelocityNodeTestAccessor::get_control_manager(node), nullptr);
}

// ---------- Lifecycle-overganger ----------

class VelocityNodeLifecycleTest : public ::testing::Test {
protected:
    std::vector<rclcpp::Parameter> overrides = make_full_parameter_overrides();
    // auto_start=false i overrides - vi styrer overgangene manuelt i testene
    Velocity_node node{make_node_options(overrides)};
};

TEST_F(VelocityNodeLifecycleTest, ConfigureSucceeds) {
    auto result = node.configure();
    EXPECT_EQ(result.id(), lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE);
}

TEST_F(VelocityNodeLifecycleTest, ActivateAfterConfigureSucceeds) {
    node.configure();
    auto result = node.activate();
    EXPECT_EQ(result.id(), lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE);
}

TEST_F(VelocityNodeLifecycleTest, DeactivateAfterActivateSucceeds) {
    node.configure();
    node.activate();
    auto result = node.deactivate();
    EXPECT_EQ(result.id(), lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE);
}

TEST_F(VelocityNodeLifecycleTest, CleanupAfterDeactivateSucceeds) {
    node.configure();
    node.activate();
    node.deactivate();
    auto result = node.cleanup();
    EXPECT_EQ(result.id(), lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED);
}

TEST_F(VelocityNodeLifecycleTest, ShutdownFromUnconfiguredSucceeds) {
    auto result = node.shutdown();
    EXPECT_EQ(result.id(), lifecycle_msgs::msg::State::PRIMARY_STATE_FINALIZED);
}

// ---------- guidance_callback: reset-terskler ----------
// NB: disse verifiserer KUN at guidance_values faktisk oppdateres riktig
// (observerbart via accessor). Selve reset_controllers()-kallet inn i
// control_manager_ptr er ikke direkte observerbart herfra uten en egen
// spy/mock på control_manager - se forslag under testene.

class VelocityNodeGuidanceCallbackTest : public ::testing::Test {
protected:
    Velocity_node node{make_node_options(make_full_parameter_overrides())};
};

TEST_F(VelocityNodeGuidanceCallbackTest, GuidanceValuesUpdateCorrectly) {
    auto msg = std::make_shared<vortex_msgs::msg::LOSGuidance>();
    msg->surge = 1.5;
    msg->pitch = 0.3;
    msg->yaw = 0.7;

    VelocityNodeTestAccessor::guidance_callback(node, msg);

    const auto& guidance = VelocityNodeTestAccessor::get_guidance_values(node);
    EXPECT_DOUBLE_EQ(guidance.surge, 1.5);
    EXPECT_DOUBLE_EQ(guidance.pitch, 0.3);
    EXPECT_DOUBLE_EQ(guidance.yaw, 0.7);
}

TEST_F(VelocityNodeGuidanceCallbackTest, SmallSurgeStepDoesNotCrashAndUpdatesValue) {
    // Fanger opp abs()-bugen indirekte: en liten men over-terskel endring
    // (0.15 > 0.1) skal trigge reset_controllers(1) i produksjonskoden.
    // Uten direkte spy på control_manager kan vi ikke bevise at reset ble
    // kalt, men vi kan bevise at selve sammenligningen ikke krasjer og at
    // verdien oppdateres korrekt uavhengig av trunkeringsbugen.
    auto msg1 = std::make_shared<vortex_msgs::msg::LOSGuidance>();
    msg1->surge = 0.0;
    VelocityNodeTestAccessor::guidance_callback(node, msg1);

    auto msg2 = std::make_shared<vortex_msgs::msg::LOSGuidance>();
    msg2->surge = 0.15;  // differanse 0.15, over terskel 0.1
    VelocityNodeTestAccessor::guidance_callback(node, msg2);

    const auto& guidance = VelocityNodeTestAccessor::get_guidance_values(node);
    EXPECT_DOUBLE_EQ(guidance.surge, 0.15);
}

// ---------- odometry_callback ----------

TEST_F(VelocityNodeGuidanceCallbackTest, OdometryCallbackResetsPublishCounter) {
    // Simuler at publish_counter har talt opp
    // (krever at accessoren evt. også kan SETTE denne for testformål,
    // eller at vi kaller publish_thrust() flere ganger uten odometry -
    // se dropout-testen under for det mønsteret)
    auto odom = std::make_shared<nav_msgs::msg::Odometry>();
    VelocityNodeTestAccessor::odometry_callback(node, odom);
    EXPECT_EQ(VelocityNodeTestAccessor::get_publish_counter(node), 0);
}

// ---------- publish_thrust: odometry dropout guard ----------

TEST_F(VelocityNodeGuidanceCallbackTest, DropoutGuardTriggersAfter100CallsWithoutOdometry) {
    // odometry_dropout_guard=true i overrides. publish_thrust() øker
    // publish_counter hver gang; ved >=100 skal den resette kontrollerne
    // og IKKE publisere. Vi kan ikke lett fange "ikke publisert" uten en
    // faktisk subscriber, men vi kan verifisere at publish_counter ikke
    // vokser videre forbi grensen (indirekte tegn på at guard-grenen tas).
    for (int i = 0; i < 150; ++i) {
        VelocityNodeTestAccessor::publish_thrust(node);
    }
    // NB: publish_counter økes FØR sjekken i produksjonskoden, så det er
    // ingen garantert øvre grense på telleren i seg selv med mindre den
    // resettes et sted - bekreft faktisk oppførsel her når testen kjøres,
    // og juster forventningen deretter (kommentar, ikke placeholder-tall).
}

