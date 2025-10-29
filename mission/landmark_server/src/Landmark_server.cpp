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

       //Filtrering function that assigns an ID to the landmarkArray, logic not implemented yet
        uint32_t id = assignID(landmark_with_id);  
        landmark_with_id.id = id;

        // Check if the ID is already stored, in that case just update position
        bool found = false;
        for (auto &stored : storedLandmarks_->landmarks)
        {
            if (stored.id == id)
            {
                
                stored.pose = landmark_with_id.pose;
                found = true;
                RCLCPP_INFO(this->get_logger(),
                            "Updated landmark #%u (type=%u, subtype=%u)",
                            id,
                            landmark_with_id.type,
                            landmark_with_id.subtype);
                break;
            }
        }

        //If the ID is new
        if (!found)
        {
            storedLandmarks_->landmarks.push_back(landmark_with_id);
            RCLCPP_INFO(this->get_logger(),
                        "Added new landmark #%u (type=%u, subtype=%u)",
                        id,
                        landmark_with_id.type,
                        landmark_with_id.subtype);
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