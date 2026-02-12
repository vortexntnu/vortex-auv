#include <pool_exploration/pool_exploration_ros.hpp>

namespace vortex::pool_exploration{

PoolExplorationNode::PoolExplorationNode(const rclcpp::NodeOptions& options)
    : Node("pool_exploration_node", options) {

    this->declare_parameter<bool>("enu_to_ned", false); //når bestemmes denne verdien?? skal være false??

    odom_frame_ =
        this->declare_parameter<std::string>("odom_frame");

    map_frame_ =
        this->declare_parameter<std::string>("map_frame");

    tf_broadcaster_ =
        std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);

    auto map_to_odom_tf = compute_map_odom_transform();

    tf_broadcaster_->sendTransform(map_to_odom_tf);

    Eigen::Affine3f eigen_transform = tf2::transformToEigen(map_to_odom_tf.transform).cast<float>();

    map_to_odom_tf_ = eigen_transform.matrix();

    double size_x = this->declare_parameter<double>("size_x", 10.0);
    double size_y = this->declare_parameter<double>("size_y", 10.0);
    double resolution = this->declare_parameter<double>("resolution", 0.1);
    std::string frame_id = this->declare_parameter<std::string>("frame_id", "map");
    
    map_ = PoolExplorationMap(size_x, size_y, resolution, frame_id);

    map_pub_ = create_publisher<nav_msgs::msg::OccupancyGrid>("map", 10);

    publish_grid();
}


void PoolExplorationNode::publish_grid()
{
    auto grid_msg = map_.grid(); //const reference
    nav_msgs::msg::OccupancyGrid msg_to_publish = grid_msg;  // make copy because const reference 
    msg_to_publish.header.stamp = this->get_clock()->now();
    map_pub_->publish(msg_to_publish);

}

geometry_msgs::msg::TransformStamped PoolExplorationNode::compute_map_odom_transform() {
    geometry_msgs::msg::TransformStamped map_to_odom;
    map_to_odom.header.stamp = this->get_clock()->now();
    map_to_odom.header.frame_id = map_frame_;
    map_to_odom.child_frame_id = odom_frame_;

    map_to_odom.transform.translation.x = 0.0;
    map_to_odom.transform.translation.y = 0.0;
    map_to_odom.transform.translation.z = 0.0;

    if (this->get_parameter("enu_to_ned").as_bool()) {
        tf2::Quaternion q1;
        q1.setRPY(M_PI, 0.0, M_PI_2);
        map_to_odom.transform.rotation.w = q1.w();
        map_to_odom.transform.rotation.x = q1.x();
        map_to_odom.transform.rotation.y = q1.y();
        map_to_odom.transform.rotation.z = q1.z();
    } else {
        //Kjøres dersom vi er i ENU frame
        tf2::Quaternion q2;
        q2.setRPY(0.0, 0.0, 0.0);
        map_to_odom.transform.rotation.w = q2.w();
        map_to_odom.transform.rotation.x = q2.x();
        map_to_odom.transform.rotation.y = q2.y();
        map_to_odom.transform.rotation.z = q2.z();
    }

    return map_to_odom;
}

}  // namespace vortex::pool_exploration
