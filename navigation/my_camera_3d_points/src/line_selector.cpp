

/*
Todo
select which line is the active line to follow
filter the lines into active and next line. Lines recieved as a pose stamped message
recieving lines as 3d endpoints 
donn| know het whhere start and end, use orientation of camera to decide with x/y coord


kham has transformed points to 3d
function that uses the 2d ponints, and sorts/groups the points to a line in camera frame
recieving either 0, 2 or 4 points for line, as poseArray
*/

#include <camera_3d_points/line_selector.hpp>
#include <camera_3d_pints/line_selector.hpp>
#include <filesystem>

using std::placeholders::_1;

namespace vortex{
namespace LineSelector {

LineSelectorNode::LineSelectorNode() : Node("line_selector_node")
    {





    }


//Needs a subscriber for line 1 and 2

// Grouping the points into a line
geometry_msgs::msg::PoseStamped line_grouper(float64 point1_, float64 point2_, 
    float64 point3_, float64 point4_)
{
    LineSeg

}


//Selecting line
geometry_msgs::msg::PoseStamped line_selector(geometry_msgs::msg::PoseStamped line1,
 geometry_msgs::msg::PoseStamped line2)
 {



 }


float64 y_





} //Namespace LineSelector


} //namespace vortex



