#include "ros/ros.h"
#include "geometry_msgs/Point.h"
#include <vortex_msgs/ObjectPosition.h>
#include <map>
#include <string>

class Landmarks{
public:
    Landmarks ():loop_rate(10) {
        op_sub = n.subscribe("object_positions_in",10, &Landmarks::callback, this);
        op_pub = n.advertise<vortex_msgs::ObjectPosition>("object_positions_out",10);
    }

    void callback(vortex_msgs::ObjectPosition objPos){
        objectPositions[objPos.objectID] = objPos.position;
        op_pub.publish(objPos);
        printMap(objectPositions);
    }

    void execute(){
        while (ros::ok()){
            ros::spinOnce();
            loop_rate.sleep();
        }
    }
    void printMap(std::map<std::string,geometry_msgs::Point> myMap){
        for(auto elem : myMap){
            ROS_INFO("ID: %s", elem.first.c_str());
            ROS_INFO("position: %f,%f,%f",elem.second.x,elem.second.y,elem.second.z);
            
        }
    }

protected:
    ros::NodeHandle n;
    ros::Subscriber op_sub;
    ros::Publisher op_pub;
    ros::Rate loop_rate;
    std::map<std::string,geometry_msgs::Point> objectPositions;
};

int main(int argc, char **argv){
    ros::init(argc,argv,"landmarks");
    Landmarks lm;
    lm.execute();
}