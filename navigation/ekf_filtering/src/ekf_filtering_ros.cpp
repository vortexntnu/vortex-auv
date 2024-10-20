#include <ekf_filtering/ekf_filtering_ros.hpp>
#include <sstream>
#include <filesystem>



/*
Functions to include
AruCo marker poses, detects and calculates position of markers
    aruco marker poses topic

Board Pose, position from centre of code, kalman filter for stable outputs
    detect_board should be set
    createRectangularBoard function -> position poblished in src/aruco_detector.hpp
    getBoardPoseStamp();

what to implement

Filtering on measurements
-> filters arbitrary measruements to desired frame


The node need to input the 6d position for the object, which is recieved from the camera
The output should be in the global frame, not local frame
*/

/// tie

using std::placeholders::_1;

namespace vortex {
namespace ekf_filtering {

EKFFilteringNode::EKFFilteringNode() : Node("ekf_filtering_node")
    {
         // Declare and acquire `target_frame` parameter
        target_frame_ = this->declare_parameter<std::string>("target_frame", "odom");

        std::chrono::duration<int> buffer_timeout(1);

        tf2_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
        // Create the timer interface before call to waitForTransform,
        // to avoid a tf2_ros::CreateTimerInterfaceException exception
        auto timer_interface = std::make_shared<tf2_ros::CreateTimerROS>(
        this->get_node_base_interface(),
        this->get_node_timers_interface());

        tf2_buffer_->setCreateTimerInterface(timer_interface);
        tf2_listener_ =
        std::make_shared<tf2_ros::TransformListener>(*tf2_buffer_);

        // Adjust the QoS to match the publisher's QoS
        rclcpp::QoS qos = rclcpp::QoS(rclcpp::KeepLast(10)).reliability(RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT);

        // Subscribe to the input PoseStamped topic
        pose_sub_.subscribe(this, "/aruco_board_pose_camera", qos.get_rmw_qos_profile());

        // Set up the MessageFilter to transform poses into the "odom" frame
        tf2_filter_ = std::make_shared<tf2_ros::MessageFilter<geometry_msgs::msg::PoseStamped>>(
            pose_sub_, *tf2_buffer_, "odom", 100, this->get_node_logging_interface(), this->get_node_clock_interface());

        tf2_filter_->registerCallback(std::bind(&EKFFilteringNode::poseCallback, this, _1));

        // Publisher for the transformed PoseStamped message
        transformed_pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/aruco_board_pose_odom", 10);

        RCLCPP_INFO(this->get_logger(), "EKFFilteringNode has been started.");
    }

void EKFFilteringNode::poseCallback(const geometry_msgs::msg::PoseStamped::ConstSharedPtr pose_msg)
    {
        // Attempt to transform the PoseStamped into the "odom" frame
        geometry_msgs::msg::PoseStamped transformed_pose;
        try {
            tf2_buffer_->transform(*pose_msg, transformed_pose, "odom", tf2::Duration(std::chrono::milliseconds(100)));
            transformed_pose_pub_->publish(transformed_pose);
            RCLCPP_INFO(this->get_logger(), "Pose transformed and published.");
        } catch (tf2::TransformException &ex) {
            RCLCPP_WARN(this->get_logger(), "Transform failed: %s", ex.what());
        }
    }

void EKFFilteringNode::kalmanFilterCallback()
{
    //Filtering the transformed pose.
    //2 publishers, one with kalman filter and one without.
    //run kalmanfiltercallback

    // static rclcpp::Time previous_time = this->now();
    // rclcpp::Time current_time = this->now();
    // //clcpp::Duration time_since_previous_callback = current_time - previous_time;
    // previous_time = current_time;
    // callback_flag = false;


    // auto [status, board_pose_meas, stamp] = board_measurement_.getBoardPoseStamp();
    
    // //if no previous measurement, previous - current = 

    // //else we have previous measurement

    // if(callback_flag == false)
    // {
    //     //run first step
    //     callback_flag = true;
    // }
    // else
    // {
    //     //run ekf step with previous steps
    //     callback_flag = false;
    //     std::tie(board_pose_est_, std::ignore, std::ignore) = EKF::step(*dynamic_model_, *sensor_model_, previous_time, board_pose_est_, board_pose_meas);
    //     break;

    // }
   
    // cv::Vec3d rvec,tvec;
    // tvec[0] = board_pose_est_.mean()(0);
    // tvec[1] = board_pose_est_.mean()(1);
    // tvec[2] = board_pose_est_.mean()(2);
    // rvec[0] = board_pose_est_.mean()(3);
    // rvec[1] = board_pose_est_.mean()(4);
    // rvec[2] = board_pose_est_.mean()(5);
    
    // tf2::Quaternion quat = rvec_to_quat(rvec);

    // geometry_msgs::msg::PoseStamped pose_msg = cv_pose_to_ros_pose_stamped(tvec, quat, frame_, stamp);
    // board_pose_pub_->publish(pose_msg);
    }


} //namespace ekf filtering
} //namespace vortex

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<vortex::ekf_filtering::EKFFilteringNode>());
    rclcpp::shutdown();
    return 0;
}

