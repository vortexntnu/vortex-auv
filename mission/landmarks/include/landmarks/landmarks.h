
#include "geometry_msgs/Point.h"
#include "geometry_msgs/PoseStamped.h"
#include "landmarks/request_position.h"
#include "ros/ros.h"
#include <map>
#include <string>
#include <vortex_msgs/ObjectPosition.h>

class Landmarks {
  /**
   * To serve as an interface between the perception system and the control
   * system, an instance of this class receives object positions ("landmarks")
   * published by the perception system on a ROS-topic. The object positions are
   * stored in a map before they are published on a ROS-topic which the control
   * system is subscribed to.
   */
public:
  Landmarks();
  /**
   * The callback function for the op_sub-subscriber.
   * @param objPose is the message received on the ROS-topic, containing an
   * object ID and a pose of the object. The object ID and pose is
   * stored in the objectPositions-map The message received is further published
   * on the object_positions_out-topic.
   */
  void callback(vortex_msgs::ObjectPosition objPose);
  /**
   * ros::spinOnce() is called at 10Hz
   */
  void execute();
  /**
   * Prints the contents of the objectPositions-map
   */
  void printMap(std::map<std::string, geometry_msgs::Point> objectsMap);

protected:
  ros::NodeHandle n;
  ros::Subscriber op_sub;
  ros::Publisher op_pub;
  ros::Rate loop_rate;
  std::map<std::string, vortex_msgs::ObjectPosition> objectPositions;
  bool send_pos(landmarks::request_position::Request &req,
                landmarks::request_position::Response &res);
  ros::ServiceServer service;
};