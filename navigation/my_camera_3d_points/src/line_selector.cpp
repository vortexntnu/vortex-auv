

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
//There is a possibility that the Y-coordinates at intersection is not at the exact same pixel
geometry_msgs::msg::PoseStamped line_grouper(float64 point1_, float64 point2_, 
    float64 point3_ = 0, float64 point4_  = 0)
{

        lines_grouped.line1_.pose.position.x = point1.x;
        lines_grouped.line1_.pose.position.y = point1.y;

        lines_grouped.line1_.end.pose.position.x = point2.x;
        lines_grouped.line1_.end.pose.position.y = point2.y;

        lines_grouped.line2_.start.pose.position.x = point3.x;
        lines_grouped.line2_.start.pose.position.y = point3.y;

        lines_grouped.line2_.end.pose.position.x = point4.x;
        lines_grouped.line2_.end.pose.position.y = point4.y;

        return lines_grouped;

}


//Selecting line based on Y values
geometry_msgs::msg::PoseStamped line_selector(lines_grouped.line1_ line1, lines_grouped.line2 line2)
 { 
    if(line1.pose.position.y > line2.pose.position.y)
    {
        return line1;
    }
    else{
        return line2;
    }
 }






} //Namespace LineSelector


} //namespace vortex



