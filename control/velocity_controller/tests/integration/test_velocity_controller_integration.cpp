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
    vortex_msgs::msg::LOSGuidance guidance_msg;
    guidance_msg.surge = 0.5;
    guidance_msg.pitch = 0.0;
    guidance_msg.yaw = 0.0;

    nav_msgs::msg::Odometry odom_msg;
    odom_msg.pose.pose.orientation.w = 1.0;

    // Publiser gjentatte ganger mens vi spinner, i stedet for én gang etter
    // en fast sleep - discovery-tiden mellom prosesser er ikke deterministisk,
    // så en engangspublisering kan lett forsvinne før subscriberen på
    // nodesiden er matchet.
    auto start = std::chrono::steady_clock::now();
    while (std::chrono::steady_clock::now() - start < 5s) {
        guidance_pub_->publish(guidance_msg);
        odom_pub_->publish(odom_msg);
        rclcpp::spin_some(test_node_);
        if (received_wrench_) break;
        std::this_thread::sleep_for(50ms);
    }

    ASSERT_TRUE(received_wrench_ != nullptr)
        << "Mottok ingen WrenchStamped innen tidsfristen - "
           "noden lastet ikke, kom ikke i ACTIVE, eller publiserte ikke";

    EXPECT_NE(received_wrench_->wrench.force.x, 0.0);
}

TEST_F(VelocityControllerIntegrationTest, StopsPublishingAfterOdometryDropout) {
    nav_msgs::msg::Odometry odom_msg;
    odom_msg.pose.pose.orientation.w = 1.0;

    // Fase 1: sørg for at noden mottar odometry og publiserer normalt
    auto start = std::chrono::steady_clock::now();
    while (std::chrono::steady_clock::now() - start < 5s) {
        odom_pub_->publish(odom_msg);
        rclcpp::spin_some(test_node_);
        if (received_wrench_) break;
        std::this_thread::sleep_for(50ms);
    }
    ASSERT_TRUE(received_wrench_ != nullptr)
        << "Fikk aldri en innledende wrench-melding - kan ikke teste dropout";

    // Fase 2: slutt å publisere odometry. Spinn KONTINUERLIG gjennom hele
    // venteperioden - selv om vi ikke bryr oss om meldingene her, må vi
    // drenere subscriber-køen fortløpende. Ellers hoper gamle meldinger
    // (publisert FØR dropout trigget) seg opp ubehandlet i QoS-bufferet
    // og blir feilaktig telt som "nye" meldinger når vi begynner å telle.
    auto dropout_deadline = std::chrono::steady_clock::now() + 12s;
    while (std::chrono::steady_clock::now() < dropout_deadline) {
        received_wrench_.reset();
        rclcpp::spin_some(test_node_);
        std::this_thread::sleep_for(20ms);
    }

    // Fase 3: nå er vi godt forbi dropout-grensen og køen er drenert.
    // Tell meldinger de neste 3 sekundene - forvent 0, siden publish_thrust()
    // returnerer tidlig uten å publisere når dropout-guarden er aktiv.
    int messages_after_dropout = 0;
    auto count_deadline = std::chrono::steady_clock::now() + 3s;
    while (std::chrono::steady_clock::now() < count_deadline) {
        received_wrench_.reset();
        rclcpp::spin_some(test_node_);
        if (received_wrench_) messages_after_dropout++;
        std::this_thread::sleep_for(20ms);
    }

    EXPECT_EQ(messages_after_dropout, 0)
        << "Noden publiserte fortsatt meldinger etter forventet dropout-grense - "
           "forvent 0, siden publish_thrust() returnerer tidlig uten å publisere "
           "når dropout-guarden trigger";
}
int main(int argc, char** argv) {
    testing::InitGoogleTest(&argc, argv);
    int result = RUN_ALL_TESTS();
    return result;
}