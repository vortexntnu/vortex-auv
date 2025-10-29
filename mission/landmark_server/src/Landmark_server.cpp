#include "rclcpp/rclcpp.hpp"
#include "vortex_msgs/msg/landmark.hpp"
#include "landmark_server/landmark_server.hpp"

using std::placeholders::_1;


LandmarkServerNode::LandmarkServerNode(const rclcpp::NodeOptions& options)
: Node("landmark_server_node", options)
{

    landmark_sub_ = this->create_subscription<vortex_msgs::msg::LandmarkArray>(
        "/landmarks",                         
        10,                                 
        std::bind(&LandmarkServerNode::landmarksRecievedCallback, this, _1)
    );

  landmarkPublisher_ = this->create_publisher<vortex_msgs::msg::LandmarkArray>(
        "/stored_landmarks", 10);


    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(100),
        std::bind(&LandmarkServerNode::publishStoredLandmarks, this));


    storedLandmarks_ = std::make_shared<vortex_msgs::msg::LandmarkArray>();

    RCLCPP_INFO(this->get_logger(), "LandmarkServerNode started and subscribed to /landmarks");

}

void LandmarkServerNode::landmarksRecievedCallback(
    const vortex_msgs::msg::LandmarkArray::SharedPtr msg)
{
    if (!storedLandmarks_) {
        storedLandmarks_ = std::make_shared<vortex_msgs::msg::LandmarkArray>();
    }

   
    for (const auto &new_landmark : msg->landmarks)
    {
        vortex_msgs::msg::Landmark landmark_with_id = new_landmark;

        
        landmark_with_id.header.frame_id = "landmark_" + std::to_string(next_id_);
        next_id_++;

   
        storedLandmarks_->landmarks.push_back(landmark_with_id);


        RCLCPP_INFO(this->get_logger(),
                    "Stored new landmark #%u (type=%u, subtype=%u, frame_id=%s)",
                    next_id_ - 1,
                    new_landmark.type,
                    new_landmark.subtype,
                    landmark_with_id.header.frame_id.c_str());
    }

    RCLCPP_INFO(this->get_logger(),
                "Total stored landmarks: %zu",
                storedLandmarks_->landmarks.size());
}



void LandmarkServerNode::publishStoredLandmarks()
{
    if (!storedLandmarks_ || storedLandmarks_->landmarks.empty()) {
        return; 
    }

    storedLandmarks_->header.stamp = this->now();
    storedLandmarks_->header.frame_id = "map";

    landmarkPublisher_->publish(*storedLandmarks_);
}