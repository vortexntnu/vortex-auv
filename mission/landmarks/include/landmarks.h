#include "ros/ros.h"
#include "geometry_msgs/Point.h"
#include <vortex_msgs/ObjectPosition.h>
#include <map>
#include <string>

class Landmarks{
    /*
    To serve as an interface between the perception system and the control system,
    an instance of this class receives object positions ("landmarks") published by 
    the perception system on a ROS-topic. The object positions are stored in a map
    before they are published on a ROS-topic which the control system is subscribed to. 
    */
public:
    Landmarks();
    void callback(vortex_msgs::ObjectPosition objPos);
    void execute();
    void printMap(std::map<std::string,geometry_msgs::Point> objectsMap);

protected:
    ros::NodeHandle n;
    ros::Subscriber op_sub;
    ros::Publisher op_pub;
    ros::Rate loop_rate;
    std::map<std::string,geometry_msgs::Point> objectPositions;
};