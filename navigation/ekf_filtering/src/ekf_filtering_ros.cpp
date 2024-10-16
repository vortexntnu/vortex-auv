#include "ekf_filtering_ros.hpp"
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
template<typename... _Elements>
constexpr tuple<_Elements&...>
tie(_Elements&... __args) noexcept
{ 
    return tuple<_Elements&...>(__args...); 
    }

void object_filter::EkfFilteringNode::kalmanFilterCallback()
{
    static rclcpp::Time previous_time = this->now();
    rclcpp::Time current_time = this->now();
    rclcpp::Duration time_since_previous_callback = current_time - previous_time;
    previous_time = current_time;

    auto [status, board_pose_meas, stamp] = board_measurement_.getBoardPoseStamp();
    switch(status) 
    {
    case BoardDetectionStatus::BOARD_NEVER_DETECTED:
        return;
    case BoardDetectionStatus::MEASUREMENT_AVAILABLE:
        std::tie(board_pose_est_, std::ignore, std::ignore) = EKF::step(*dynamic_model_, *sensor_model_, time_since_previous_callback.seconds(),board_pose_est_, board_pose_meas);
        break;
   
    cv::Vec3d rvec,tvec;
    tvec[0] = board_pose_est_.mean()(0);
    tvec[1] = board_pose_est_.mean()(1);
    tvec[2] = board_pose_est_.mean()(2);
    rvec[0] = board_pose_est_.mean()(3);
    rvec[1] = board_pose_est_.mean()(4);
    rvec[2] = board_pose_est_.mean()(5);
    
    tf2::Quaternion quat = rvec_to_quat(rvec);

    geometry_msgs::msg::PoseStamped pose_msg = cv_pose_to_ros_pose_stamped(tvec, quat, frame_, stamp);
    board_pose_pub_->publish(pose_msg);
    }
}


