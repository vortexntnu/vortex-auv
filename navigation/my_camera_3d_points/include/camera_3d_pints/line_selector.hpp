#ifndef LINE_SELECTOR_HPP
#define LINE_SELECTOR_HPP
#include "../../src/camera_3d_points_node.cpp"



#include <chrono>
#include <memory>
#include <string>
#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/set_bool.hpp>

#include "geometry_msgs/msg/point_stamped.hpp"
#include "geometry_msgs/msg/pose_array.hpp"
#include "message_filters/subscriber.h"
#include "rclcpp/parameter_event_handler.hpp"
#include <rclcpp/qos.hpp>
#include "tf2_ros/buffer.h"
#include "tf2_ros/create_timer_ros.h"
#include "tf2_ros/message_filter.h"
#include "tf2_ros/transform_listener.h"
#include <tuple>
#include <Eigen/Dense>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <vortex_filtering/vortex_filtering.hpp>



namespace vortex::LineSelector

{
using Vector6d = Eigen::Vector<double, 6>;

class LineSelectorNode :: public rllcpp::Node{

    public:
        LineSelectorNode();

        LineSelectorNode(){};

    private:
         
        //Struct to store two points in a line

        struct PointGrouper
        {
            geometry_msgs::msg::Point start;
            geometry_msgs::msg::Point end;
        };

        //struct to store both lines together
        struct LineGrouper
        {
         PointGrouper line1_;
         PointGrouper line2_;
        };

        LineGrouper lines_combined;

        //selects which line is the current one
        LineGrouper line_selector(lines_combined.line1_ line1, lines_combined.line2 line2);


        //function for grouping two points into a line
        //might have to run recursive
        LineGrouper lines_grouped(double point1_, double point2_, 
            double point3_ = 0, double point4_  = 0);



        //compare the y coordinate of the two lines
        // float64 y_coor_line_comp_(float64 line1_y_, float64 line2_y_);


} //Line selector node









} // Line selector namespace
