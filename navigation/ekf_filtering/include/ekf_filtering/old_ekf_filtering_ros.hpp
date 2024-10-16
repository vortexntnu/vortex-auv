#ifndef EKF_FILTERING_ROS_HPP
#define EKF_FILTERING_ROS_HPP

#include <rclcpp/rclcpp.hpp>

#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include "rclcpp/parameter_event_handler.hpp"
#include <rclcpp/qos.hpp>
#include <tuple>

#include <map>
#include <string>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <Eigen/Dense>
#include <vortex_filtering/vortex_filtering.hpp>

#include <cv_bridge/cv_bridge.h>

#include "aruco_detector.hpp"

namespace vortex::ekf_filtering
{
using Vector6d = Eigen::Vector<double, 6>;

static std::map<std::string, cv::aruco::PREDEFINED_DICTIONARY_NAME> dictionary_map = {
        {"DICT_4X4_50", cv::aruco::DICT_4X4_50},
        {"DICT_4X4_100", cv::aruco::DICT_4X4_100},
        {"DICT_4X4_250", cv::aruco::DICT_4X4_250},
        {"DICT_4X4_1000", cv::aruco::DICT_4X4_1000},
        {"DICT_5X5_50", cv::aruco::DICT_5X5_50},
        {"DICT_5X5_100", cv::aruco::DICT_5X5_100},
        {"DICT_5X5_250", cv::aruco::DICT_5X5_250},
        {"DICT_5X5_1000", cv::aruco::DICT_5X5_1000},
        {"DICT_6X6_50", cv::aruco::DICT_6X6_50},
        {"DICT_6X6_100", cv::aruco::DICT_6X6_100},
        {"DICT_6X6_250", cv::aruco::DICT_6X6_250},
        {"DICT_6X6_1000", cv::aruco::DICT_6X6_1000},
        {"DICT_7X7_50", cv::aruco::DICT_7X7_50},
        {"DICT_7X7_100", cv::aruco::DICT_7X7_100},
        {"DICT_7X7_250", cv::aruco::DICT_7X7_250},
        {"DICT_7X7_1000", cv::aruco::DICT_7X7_1000},
        {"DICT_ARUCO_ORIGINAL", cv::aruco::DICT_ARUCO_ORIGINAL}};

/**
 * @class ArucoDetectorNode
 * @brief ROS node for ArUco marker detection and pose estimation.
 * 
 * This class represents a ROS node that performs ArUco marker detection and pose estimation. Also supports detection of ArUco boards.
 * It subscribes to image and camera info topics, and publishes marker poses, marker images, and board poses.
 * It also provides functionalities for setting camera parameters, initializing the detector, setting visualization options,
 * setting board detection options, initializing the board, initializing the models, and handling parameter events.
 */

class ekf_filtering : public rclcpp:Node
{
public:
    EkfFilteringNode



}




class ArucoDetectorNode : public rclcpp::Node{

public:
    /**
     * @brief Constructs an ArucoDetectorNode object.
     */
    ArucoDetectorNode(const rclcpp::NodeOptions & options);

    /**
     * @brief Destroys the ArucoDetector object.
     */
    ~ArucoDetectorNode(){};

    
private: 
    /**
     * @brief Subscribes to image topic
    */
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
    /**
     * @brief Subscribes to camera info topic to retrieve and set camera parameters
    */
    rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_sub_;

    /**
     * @brief Publishes marker poses as a PoseArray
    */
    rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr marker_pose_pub_;
    /**
     * @brief Publishes the image with the markers visualized if visualization param is set. visualizes the board pose if board detection is set.
    */
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr marker_image_pub_;
    /**
     * @brief Publishes the pose of the board
    */
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr board_pose_pub_;

    /**
     * @brief Retrieves initial camera parameters from ros parameters and sets them. 
     * Will most likely be overwritten by params from camera_info topic.
    */
    void setCameraParams();

    /**
     * @brief Initialize the detector. Sets dictionary from ros param. Also initializes detector parameters.
    */
    void initializeDetector();

    /**
     * @brief Sets visualization flag from ros param
    */
    void setVisualization();

    /**
     * @brief Set board detection flag from ros param
    */
    void setBoardDetection();

    /**
     * @brief Initializes board from ros params
    */
    void initializeBoard();

    /**
     * @brief Initializes dynamic and sensor models with std_dev from ros params
    */
    void initializeModels();

    /**
     * @brief Function to toggle the kalman filter on/off, based on detect_board ros param and if the timer is active
    */
    void toggleKalmanFilterCallback();

    /**
     * @brief Check and subscribe to camera topics if not yet subscribed. Allows for dynaminc reconfiguration of camera topics.
     * If new topics are set, the old subscriptions are cancelled and new ones are bound to the callback functions.
     * 
    */
    void checkAndSubscribeToCameraTopics();

    /**
     * @brief Set the frame to use for transforms to get correct pose of markers and board.
    */
    void setFrame();
    
    /**
     * @brief Initialize the parameter handler and a parameter event callback.
     * 
    */
    void initializeParameterHandler();
    /**
     * @brief Callback function for parameter events.
     * Checks for parameter changes that matches the nodes' namespace and invokes the relevant initializer functions to update member variables.
     * 
     * @param event The parameter event.
    */  
    void onParameterEvent(const rcl_interfaces::msg::ParameterEvent &event);

    /**
     * @brief Manages parameter events for the node.
     *
     * This handle is used to set up a mechanism to listen for and react to changes in parameters. 
     * Parameters can be used to configure the node's operational behavior dynamically, 
     * allowing adjustments without altering the code. The `param_handler_` is responsible for 
     * registering callbacks that are triggered on parameter changes, providing a centralized 
     * management system within the node for such events.
     */
    std::shared_ptr<rclcpp::ParameterEventHandler> param_handler_;


    /**
     * @brief Handle to the registration of the parameter event callback.
     *
     * Represents a token or reference to the specific callback registration made with 
     * the parameter event handler (`param_handler_`). This handle allows for management 
     * of the lifecycle of the callback, such as removing the callback if it's no longer needed. 
     * It ensures that the node can respond to parameter changes with the registered callback 
     * in an efficient and controlled manner.
     */
    rclcpp::ParameterEventCallbackHandle::SharedPtr param_cb_handle_;
    
    /**
     * @brief Callback function for image topic
     * 
     * @param msg The image message
    */
    void imageCallback(const sensor_msgs::msg::Image::SharedPtr msg);

    /**
     * @brief Callback function for camera info topic.
     *  Sets camera matrix and distortion coefficients from camera info message and logs the params.
     * 
     * @param msg The camera info message
    */
    void cameraInfoCallback(const sensor_msgs::msg::CameraInfo::SharedPtr msg);

    /**
     * @brief Timer callback function for the kalman filter to estimate the board pose.
     *  Gives a more stable output of board pose benefitial for control applications.
    */
    void kalmanFilterCallback();

    /**
     * @brief Function to convert a rotation vector to a quaternion
    */
    tf2::Quaternion rvec_to_quat(const cv::Vec3d &rvec);

    /**
     * @brief Function to convert a translation vector and quaternion to a PoseStamped message
    */
    geometry_msgs::msg::PoseStamped cv_pose_to_ros_pose_stamped(const cv::Vec3d &tvec, const tf2::Quaternion &quat, std::string frame_id, rclcpp::Time stamp);


    using DynMod = vortex::models::IdentityDynamicModel<6>;
    using SensMod = vortex::models::IdentitySensorModel<6,6>;
    using EKF = vortex::filter::EKF<DynMod, SensMod>;

    enum class BoardDetectionStatus{
        BOARD_NEVER_DETECTED,
        MEASUREMENT_AVAILABLE,
        MEASUREMENT_NOT_AVAILABLE,
    };

    /**
     * @brief Struct to store the board pose, status and timestamp using in kalmanFilterCallback function.
    */
    struct BoardPoseStamp{
    std::tuple<BoardDetectionStatus, Vector6d, rclcpp::Time> getBoardPoseStamp() const {
        return {board_detection_status_, board_pose_meas_, stamp_};
    }

    /**
     * @brief Sets the board status. Used when board is not detected 
     * to set status to MEASUREMENT_NOT_AVAILABLE
     *  
     * 
     * @param status The board status
    */
    void setBoardStatus(BoardDetectionStatus status) {
        board_detection_status_ = status;
    }
    
    /**
     * @brief Sets all variables of the struct simultaneously to ensure thread safety
     *      when retrieved in kalmanFilterCallback.
     *  
     * 
     * 
     * @param values A tuple containing the board status, pose and timestamp
    */
    void setBoardPoseStamp(const std::tuple<BoardDetectionStatus, Vector6d, rclcpp::Time>& values) {
        board_detection_status_ = std::get<0>(values);
        board_pose_meas_ = std::get<1>(values);
        stamp_ = std::get<2>(values);
    }
    
    private:
        BoardDetectionStatus board_detection_status_ = BoardDetectionStatus::BOARD_NEVER_DETECTED;
        Vector6d board_pose_meas_ = Vector6d::Zero(6); 
        rclcpp::Time stamp_;
    };
    
    std::unique_ptr<ArucoDetector> aruco_detector_;

    cv::Mat camera_matrix_, distortion_coefficients_;

    bool detect_board_;
    bool visualize_;

    float marker_size_;
    float xDist_,yDist_;
    std::vector<int64_t> ids_;
    std::vector<int> ids_detected_once_;
    std::vector<int> ids_detected_secure_;
    std::unordered_map<int,int> id_detection_counter_;
    cv::Ptr<cv::aruco::Dictionary> dictionary_;
    std::string frame_;
    cv::Ptr<cv::aruco::DetectorParameters> detector_params_;
    cv::Ptr<cv::aruco::Board> board_;
    BoardPoseStamp board_measurement_;
    std::shared_ptr<DynMod> dynamic_model_;
    std::shared_ptr<SensMod> sensor_model_;
    vortex::prob::Gauss<6> board_pose_est_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::TimerBase::SharedPtr timeout_timer_;
    std::string image_topic_;
    std::string camera_info_topic_;

};
} // namespace aruco_detector

#endif //ARUCO_DETECTOR_ROS_HPP