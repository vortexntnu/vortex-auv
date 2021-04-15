#ifndef DEPTH_ESTIMATOR_H
#define DEPTH_ESTIMATOR_H


#include <ros/ros.h>
#include <ros/console.h>
#include <std_msgs/Float64.h>


class DepthEstimator
{
    public:
    DepthEstimator();

    private:
    pressure_sub;

};


#endif
