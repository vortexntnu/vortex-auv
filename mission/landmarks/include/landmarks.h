#include "ros/ros.h"
#include "geometry_msgs/Point.h"
#include <vortex_msgs/ObjectPosition.h>
#include <map>
#include <string>

class Landmarks{
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