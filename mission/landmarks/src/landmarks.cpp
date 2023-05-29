#include "landmarks/landmarks.h"

Landmarks::Landmarks() : loop_rate(10) {
  op_sub = n.subscribe("object_positions_in", 10, &Landmarks::callback, this);
  op_pub = n.advertise<vortex_msgs::ObjectPosition>("object_positions_out", 10);
  service = n.advertiseService("send_positions", &Landmarks::send_pos, this);
  geometry_msgs::Point p;
  vortex_msgs::ObjectPosition obj;
  obj.objectPose.pose.position = p;
  obj.isDetected = false;
  obj.estimateConverged = false;
  obj.estimateFucked = false;
  objectPositions["gate"] = obj;
  objectPositions["pole"] = obj;
  objectPositions["path"] = obj;
  objectPositions["buoy"] = obj;
  objectPositions["recovery_point"] = obj;
  objectPositions["docking_point"] = obj;
}

void Landmarks::callback(vortex_msgs::ObjectPosition objPose) {
  objectPositions[objPose.objectID] = objPose;
  op_pub.publish(objPose);
}

void Landmarks::execute() {
  while (ros::ok()) {
    ros::spinOnce();
    loop_rate.sleep();
  }
}

void Landmarks::printMap(
    std::map<std::string, geometry_msgs::Point> objectsMap) {
  for (auto elem : objectsMap) {
    ROS_INFO("ID: %s", elem.first.c_str());
    ROS_INFO("position: %f,%f,%f", elem.second.x, elem.second.y, elem.second.z);
  }
}

bool Landmarks::send_pos(landmarks::request_position::Request &req,
                         landmarks::request_position::Response &res) {
  res.object = Landmarks::objectPositions[req.ID];
  return true;
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "landmarks");
  Landmarks lm;
  lm.execute();
}