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

void LandmarkServerNode::landmarksReceivedCallback(
    const vortex_msgs::msg::LandmarkArray::SharedPtr msg)
{
    std::lock_guard<std::mutex> lock(storedLandmarksMutex_);

    for (auto &landmark : msg->landmarks)
    {
        auto landmark_with_id = landmark;

        // Assign unique ID (logic not implemented yet)
        uint32_t id = assignID(landmark_with_id);
        landmark_with_id.id = id;

        // Check if already stored
        auto it = std::find_if(
            storedLandmarks_->landmarks.begin(),
            storedLandmarks_->landmarks.end(),
            [id](const auto &stored) { return stored.id == id; });

        if (it != storedLandmarks_->landmarks.end()) {
            it->pose = landmark_with_id.pose;
            RCLCPP_INFO(this->get_logger(),
                        "Updated landmark #%u (type=%u, subtype=%u)",
                        id, landmark_with_id.type, landmark_with_id.subtype);



        } else {
            storedLandmarks_->landmarks.push_back(landmark_with_id);
            RCLCPP_INFO(this->get_logger(),
                        "Added new landmark #%u (type=%u, subtype=%u)",
                        id, landmark_with_id.type, landmark_with_id.subtype);
        }
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