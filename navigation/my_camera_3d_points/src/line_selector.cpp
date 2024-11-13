

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
LineGrouper lines_grouped(double point1_, double point2_, 
    double point3_ = 0, double point4_  = 0)
{
        LineGrouper lines_grouped_temp;

        lines_grouped.line1_.start.x= point1.x;
        lines_grouped.line1_.start.y = point1.y;

        lines_grouped.line1_.end.x = point2.x;
        lines_grouped.line1_.end.y = point2.y;

        lines_grouped.line2_.start.x = point3.x;
        lines_grouped.line2_.start.y = point3.y;

        lines_grouped.line2_.end.x = point4.x;
        lines_grouped.line2_.end.y = point4.y;

        return lines_grouped_temp;

}


//Selecting line based on Y values
geometry_msgs::msg::PoseStamped line_selector(lines_grouped.line1_ line1, lines_grouped.line2 line2)
 { 
    if(line1.start.y > line2.end.y)
    {
        return line1;
    }
    else{
        return line2;
    }
 }






} //Namespace LineSelector


} //namespace vortex



