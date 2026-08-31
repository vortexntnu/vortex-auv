#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/wrench_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <vortex_msgs/msg/los_guidance.hpp>
#include <chrono>

using namespace std::chrono_literals;

class VelocityControllerIntegrationTest : public ::testing::Test {
protected:
    static void SetUpTestSuite() {
        rclcpp::init(0, nullptr);
    }
    static void TearDownTestSuite() {
        rclcpp::shutdown();
    }

    void SetUp() override {
    test_node_ = std::make_shared<rclcpp::Node>("integration_test_client");

    received_wrench_.reset();

    rclcpp::QoS wrench_qos(10);
    wrench_qos.best_effort().durability_volatile();

    wrench_sub_ = test_node_->create_subscription<geometry_msgs::msg::WrenchStamped>(
        "/nautilus/wrench_input", wrench_qos,
        [this](geometry_msgs::msg::WrenchStamped::SharedPtr msg) {
            received_wrench_ = msg;
        });

    rclcpp::QoS input_qos(10);
    input_qos.best_effort().durability_volatile();

    guidance_pub_ = test_node_->create_publisher<vortex_msgs::msg::LOSGuidance>(
        "/nautilus/guidance/los", input_qos);
    odom_pub_ = test_node_->create_publisher<nav_msgs::msg::Odometry>(
        "/nautilus/odom", input_qos);
}

    // Hjelper: spinn i inntil timeout, avbryt tidlig hvis received_wrench_ er satt
    bool spin_until_wrench_received(std::chrono::milliseconds timeout) {
        auto start = std::chrono::steady_clock::now();
        while (std::chrono::steady_clock::now() - start < timeout) {
            rclcpp::spin_some(test_node_);
            if (received_wrench_) return true;
            std::this_thread::sleep_for(10ms);
        }
        return false;
    }

    std::shared_ptr<rclcpp::Node> test_node_;
    rclcpp::Subscription<geometry_msgs::msg::WrenchStamped>::SharedPtr wrench_sub_;
    rclcpp::Publisher<vortex_msgs::msg::LOSGuidance>::SharedPtr guidance_pub_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
    geometry_msgs::msg::WrenchStamped::SharedPtr received_wrench_;
};

TEST_F(VelocityControllerIntegrationTest, NodeLoadsAndPublishesThrustAfterOdometry) {
    // NB: krever at velocity_controller_node allerede er startet av launch-filen
    // og at auto_start=true i params, slik at noden selv går til ACTIVE.
    // Gi den litt tid til å komme opp og konfigurere seg via lifecycle-timeren.
    std::this_thread::sleep_for(500ms);

    vortex_msgs::msg::LOSGuidance guidance_msg;
    guidance_msg.surge = 0.5;
    guidance_msg.pitch = 0.0;
    guidance_msg.yaw = 0.0;
    guidance_pub_->publish(guidance_msg);

    nav_msgs::msg::Odometry odom_msg;
    odom_msg.pose.pose.orientation.w = 1.0;  // identitet
    odom_pub_->publish(odom_msg);

    ASSERT_TRUE(spin_until_wrench_received(5000ms))
        << "Mottok ingen WrenchStamped innen tidsfristen - "
           "noden lastet ikke, kom ikke i ACTIVE, eller publiserte ikke";

    // Grov sanity-sjekk: positiv surge-referanse bør gi et ikke-null pådrag i x
    EXPECT_NE(received_wrench_->wrench.force.x, 0.0);
}

TEST_F(VelocityControllerIntegrationTest, StopsPublishingMeaningfulThrustAfterOdometryDropout) {
    std::this_thread::sleep_for(500ms);

    nav_msgs::msg::Odometry odom_msg;
    odom_msg.pose.pose.orientation.w = 1.0;
    odom_pub_->publish(odom_msg);
    spin_until_wrench_received(2000ms);

    // Slutt å publisere odometry. Spinn KONTINUERLIG i stedet for én lang
    // sleep + ett spin_some-kall, slik at vi ser hver melding i rekkefølge
    // og fanger den faktiske overgangen til nullwrench, i stedet for å
    // risikere å plukke opp en gammel bufret melding fra før dropout-grensen.
    auto start = std::chrono::steady_clock::now();
    bool saw_zero_thrust = false;
    while (std::chrono::steady_clock::now() - start < 12s) {
        rclcpp::spin_some(test_node_);
        if (received_wrench_ &&
            received_wrench_->wrench.force.x == 0.0 &&
            received_wrench_->wrench.torque.y == 0.0 &&
            received_wrench_->wrench.torque.z == 0.0) {
            saw_zero_thrust = true;
            break;
        }
        std::this_thread::sleep_for(10ms);
    }

    EXPECT_TRUE(saw_zero_thrust)
        << "Forventet å motta et nullwrench etter odometry-dropout innen 12s, "
           "men siste mottatte verdi var force.x="
        << (received_wrench_ ? received_wrench_->wrench.force.x : -999.0);
}

int main(int argc, char** argv) {
    testing::InitGoogleTest(&argc, argv);
    int result = RUN_ALL_TESTS();
    return result;
}