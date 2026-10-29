#ifndef POOL_EXPLORATION_ROS_HPP
#define POOL_EXPLORATION_ROS_HPP

#include <memory>

#include <Eigen/Core>

#include <rclcpp/rclcpp.hpp>

#include <message_filters/subscriber.h>

#include <tf2_ros/buffer.h>
#include <tf2_ros/message_filter.h>
#include <tf2_ros/transform_listener.h>

#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>

#include <vortex_msgs/msg/line_segment2_d_array.hpp>
#include <vortex_msgs/msg/sonar_info.hpp>
#include <vortex_msgs/srv/send_waypoints.hpp>

#include <vortex/utils/types.hpp> 

#include <pool_exploration/pool_exploration.hpp>
#include <visualization_msgs/msg/marker.hpp> //til testing

namespace vortex::pool_exploration{

/**
 * @brief ROS 2 node for pool exploration and docking waypoint generation.
 *
 * This node provides the ROS interface for the pool exploration planner.
 * It subscribes to detected sonar line segments, vehicle pose, and sonar
 * image metadata, transforms the incoming line segments into the planning
 * frame, estimates a valid pool corner through the planner, computes a
 * docking position, and sends the resulting waypoint to the waypoint manager.
 */
 
class PoolExplorationNode : public rclcpp::Node {
    public:
        /**
        * @brief Construct a new PoolExplorationNode.
        *
        * The constructor initializes parameters, ROS interfaces, and the planner.
        *
        * @param options ROS node options.
        */
        explicit PoolExplorationNode(const rclcpp::NodeOptions& options);

        /** @brief Default destructor. */
        ~PoolExplorationNode() = default;

    private:
        // setup

        /**
        * @brief Declare and load ROS parameters used by the node.
        *
        * This includes frame names, topic names, and waypoint-related parameters.
        */
        void setup_parameters();

        /**
        * @brief Create subscribers, TF utilities, and service clients.
        *
        * This method initializes:
        * - TF buffer and listener,
        * - line, pose, and sonar-info subscriptions,
        * - TF message filter for line detections,
        * - waypoint service client.
        */
        void setup_publishers_and_subscribers();

        /**
        * @brief Create and configure the pool exploration planner.
        *
        * Planner configuration values are loaded from ROS parameters and used to
        * construct the internal planner instance.
        */
        void setup_planner();

        // callbacks

        /**
        * @brief Callback for vehicle pose updates.
        *
        * Updates the internal vehicle state estimate used by the planner.
        *
        * @param msg Vehicle pose message with covariance.
        */
        void pose_callback(
            const geometry_msgs::msg::PoseWithCovarianceStamped::ConstSharedPtr& msg);

        /**
        * @brief Callback for sonar image metadata updates.
        *
        * Stores the most recent sonar information needed to convert detected line
        * endpoints from image pixels to metric sonar coordinates.
        *
        * @param msg Sonar metadata message.
        */
        void sonar_info_callback(
            const vortex_msgs::msg::SonarInfo::ConstSharedPtr& msg);
        
        /**
        * @brief Callback for detected line segments.
        *
        * Triggered when a line segment message passes the TF message filter. The
        * callback attempts to estimate a docking waypoint from the detected lines.
        *
        * @param msg Array of detected 2D line segments.
        */
        void line_callback(
            const vortex_msgs::msg::LineSegment2DArray::ConstSharedPtr& msg);

        // helpers 

        /**
        * @brief Estimate and send a docking waypoint from detected line segments.
        *
        * The input segments are transformed into the planning frame, passed to the
        * planner for corner detection, and then used to estimate a docking point.
        * If a valid estimate is found, a waypoint is sent.
        *
        * @param msg Array of detected 2D line segments.
        */
        void estimate_and_send_docking_waypoint(const vortex_msgs::msg::LineSegment2DArray& msg);
        
        /**
        * @brief Transform sonar-detected 2D segments into a target frame.
        *
        * Each line endpoint is first converted from pixel coordinates to sonar
        * metric coordinates using the latest sonar metadata. The resulting points
        * are then transformed by the provided homogeneous transform.
        *
        * @param msg Input line segment array.
        * @param T_target_src Homogeneous transform from source frame to target frame.
        * @return Vector of transformed 2D line segments in the target frame.
        */
        std::vector<vortex::utils::types::LineSegment2D> transform_segments_2d(
            const vortex_msgs::msg::LineSegment2DArray& msg,
            const Eigen::Matrix4f& T_target_src); 

        /**
        * @brief Send a docking waypoint to the waypoint manager.
        *
        * A single waypoint is created from the estimated docking position and sent
        * through the SendWaypoints service. The request uses the configured waypoint
        * control parameters.
        *
        * @param docking_estimate Estimated docking position in the planning frame.
        */
        void send_docking_waypoint(const Eigen::Vector2f& docking_estimate);

        //TIL TESTING
        void publish_docking_marker(const Eigen::Vector2f& docking);
        

        /** @brief Topic name for detected sonar line segments. */
        std::string line_sub_topic_;

        /** @brief Topic name for vehicle pose estimates. */
        std::string pose_sub_topic_;

        /** @brief Topic name for sonar image metadata. */
        std::string sonar_info_sub_topic_;

        // TIL TESTING
        std::string debug_topic_;


        /** @brief Odometry frame used as planning/reference frame. */
        std::string odom_frame_;

        /** @brief Vehicle base frame name. */
        std::string base_frame_;

        /** @brief Sonar frame name. */
        std::string sonar_frame_;


        /** @brief Distance threshold for switching to the next waypoint [m]. */
        float waypoint_switching_threshold_;

        /** @brief Whether newly sent waypoints should overwrite previously queued waypoints. */
        bool waypoint_overwrite_prior_;

        /** @brief Whether the sent waypoint should take priority in the waypoint manager. */
        bool waypoint_take_priority_;


        /** @brief True once a docking waypoint has been sent. */
        bool waypoint_sent_{false};

        /** @brief Latest known vehicle pose and heading used by the planner. */
        vortex::utils::types::PoseEuler drone_state_;

        /** @brief Most recent sonar metadata message. */
        vortex_msgs::msg::SonarInfo::ConstSharedPtr latest_sonar_info_;


        /** @brief TF buffer used for transform lookup. */
        std::shared_ptr<tf2_ros::Buffer> tf_buffer_;

        /** @brief TF listener attached to the TF buffer. */
        std::shared_ptr<tf2_ros::TransformListener> tf_listener_;


    /** @brief Subscriber for line segment detections. */
    message_filters::Subscriber<vortex_msgs::msg::LineSegment2DArray> line_sub_;

    /**
     * @brief TF-aware message filter for line segment detections.
     *
     * Ensures that line messages are only forwarded once the required transform
     * into the target frame is available.
     */
    std::shared_ptr<tf2_ros::MessageFilter<vortex_msgs::msg::LineSegment2DArray>> line_filter_;

    /** @brief Subscriber for vehicle pose updates. */
    rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pose_sub_;

    /** @brief Subscriber for sonar metadata updates. */
    rclcpp::Subscription<vortex_msgs::msg::SonarInfo>::SharedPtr sonar_info_sub_;


    /** @brief Client for sending waypoint requests to the waypoint manager. */
    rclcpp::Client<vortex_msgs::srv::SendWaypoints>::SharedPtr waypoint_client_;


    /** @brief Pool exploration planner used for corner detection and docking estimation. */
    std::unique_ptr<PoolExplorationPlanner> planner_;

    // Debug visualization
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr docking_marker_pub_; //til testing
};

// GRID LOGIKK TIL SENERE
# if 0
#include <tf2_ros/static_transform_broadcaster.h> 

    private:
                
        std::string map_pub_topic_;
        std::string map_frame_;

        std::shared_ptr<tf2_ros::StaticTransformBroadcaster> tf_broadcaster_;
      
        // Timer
        rclcpp::TimerBase::SharedPtr timer_;

        std::chrono::milliseconds pub_dt_{200};

        void timer_callback();
        // Lager en transformasjon TF map->odom
        geometry_msgs::msg::TransformStamped compute_map_odom_transform();
        
        void publish_grid();

        // Logikk til senere bruk:
        void draw_segments_in_map_frame(const vortex_msgs::msg::LineSegment2DArray& msg);

        // publisher
        rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr 
            map_pub_;

        // map
        PoolExplorationPlanner map_;

#endif
}  // namespace vortex::pool_exploration

#endif  // POOL_EXPLORATION_ROS_HPP

