#ifndef LINE_FILTERING__ROS__LINE_FILTERING_ROS_HPP_
#define LINE_FILTERING__ROS__LINE_FILTERING_ROS_HPP_

#include <message_filters/subscriber.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/create_timer_ros.h>
#include <tf2_ros/message_filter.h>
#include <tf2_ros/transform_listener.h>

#include <chrono>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <string>
#include <vector>

#include <geometry_msgs/msg/point_stamped.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <vortex_msgs/msg/line_segment2_d_array.hpp>
#include <vortex_msgs/msg/line_segment3_d_array.hpp>
#include <vortex_msgs/msg/sonar_info.hpp>

#include "line_filtering/lib/line_track_manager.hpp"

namespace vortex::line_filtering {

class LineFilteringNode : public rclcpp::Node {
   public:
    explicit LineFilteringNode(const rclcpp::NodeOptions& options);
    ~LineFilteringNode() = default;

   private:
    void setup_publishers_and_subscribers();
    void setup_track_manager();

    /**
     * @brief Called when SonarInfo arrives – caches the latest sonar metadata.
     */
    void sonar_info_callback(
        const vortex_msgs::msg::SonarInfo::ConstSharedPtr& msg);

    /**
     * @brief Called when LineSegment2DArray arrives.
     *
     * 1. Convert pixel line segments to metric 2D points using SonarInfo.
     * 2. Treat them as planar xy (z=0) in the sonar frame.
     * 3. TF-lookup sonar_frame -> target_frame, transform endpoints to 3D.
     * 4. Project the 3D endpoints onto the odom xy-plane to get 2D line
     *    segments.
     * 5. Convert to LineMeasurements and store.
     */
    void line_segment_callback(
        const vortex_msgs::msg::LineSegment2DArray::ConstSharedPtr& msg);

    /**
     * @brief Timer-driven filter step.
     */
    void timer_callback();

    /**
     * @brief Publish confirmed tracks as LineSegment3DArray (z=0, in odom).
     */
    void publish_tracks();

    /**
     * @brief Publish visualization markers for rviz.
     */
    void publish_markers();

    /**
     * @brief Create debug publishers (raw measurement markers + filtered
     * markers).
     */
    void setup_debug_publishers();

    /**
     * @brief Publish raw (pre-filter) line measurements as red markers.
     */
    void publish_debug_meas();

    /**
     * @brief Publish filtered line tracks as green markers.
     */
    void publish_debug_state();

    std::string target_frame_;
    std::string sonar_frame_;
    std::shared_ptr<tf2_ros::Buffer> tf2_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf2_listener_;

    using LineMsgT = vortex_msgs::msg::LineSegment2DArray;

    /**
     * @brief Create the message-filter subscription for line segments.
     *        The callback only fires when TF to target_frame_ is available.
     */
    void create_line_subscription(const std::string& topic_name,
                                  const rmw_qos_profile_t& qos_profile);

    std::shared_ptr<message_filters::Subscriber<LineMsgT>> line_sub_;
    std::shared_ptr<tf2_ros::MessageFilter<LineMsgT>> tf_filter_;

    rclcpp::Subscription<vortex_msgs::msg::SonarInfo>::SharedPtr
        sonar_info_sub_;

    rclcpp::Publisher<vortex_msgs::msg::LineSegment3DArray>::SharedPtr
        line_pub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr
        marker_pub_;

    rclcpp::TimerBase::SharedPtr pub_timer_;
    std::chrono::milliseconds filter_dt_{0};

    std::unique_ptr<LineTrackManager> track_manager_;
    std::vector<LineMeasurement> measurements_;

    vortex_msgs::msg::SonarInfo::ConstSharedPtr latest_sonar_info_;

    bool debug_{false};

    struct OdomSegment {
        Eigen::Vector2d p0;
        Eigen::Vector2d p1;
    };
    std::vector<OdomSegment> debug_odom_segments_;

    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr
        debug_meas_marker_pub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr
        debug_state_marker_pub_;
};

}  // namespace vortex::line_filtering

#endif  // LINE_FILTERING__ROS__LINE_FILTERING_ROS_HPP_
