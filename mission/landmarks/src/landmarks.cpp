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
    }

    void execute(){
        while (ros::ok()){
            ros::spinOnce();
            loop_rate.sleep();
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
    cpi.execute();
}