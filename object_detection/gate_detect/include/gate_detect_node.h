
#ifndef GATE_DETECT_NODE_H
#define GATE_DETECT_NODE_H

/*   Written by Ambjørn Grimsrud Waldum, Student
     Edited by Kristoffer Rakstad Solberg, Student
     Copyright (c) 2020 Manta AUV, Vortex NTNU.
     All rights reserved. */

// Dependencies
#include <opencv2/opencv.hpp>
#include <iostream>
#include <list>
#include <vector>
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "std_msgs/Bool.h"
#include "vortex_msgs/CameraObjectInfo.h"
#include <sstream>
#include <ros/ros.h>
#include <dynamic_reconfigure/server.h>
#include <gate_detect/GateParamsConfig.h>

// Quality of life
using namespace cv;
using namespace std;
using namespace cv_bridge;
using namespace image_transport;
static const std::string OPENCV_WINDOW = "Binary image, gate";
static const std::string WINDOW2 = "Bounding boxes, gate";

class gateFinder{

private:

  // Setting up ROS handles, subsribers and publishers	
  ros::NodeHandle nh_;
  ros::NodeHandle n_;
  ImageTransport it_;

  // ros topics
  Subscriber image_sub_;
  Publisher image_pub_;
  Publisher gate_pub_;
  ros::Publisher detect_pub_;

  //Dynamic reconfigure
  dynamic_reconfigure::Server<gate_detect::GateParamsConfig> server;
  dynamic_reconfigure::Server<gate_detect::GateParamsConfig>::CallbackType f;

  // hsv variables
  int minhue,maxhue,minval,maxval,minsat,maxsat;

  // Gate height treshold for detection
  double height;

  // image frame coordinates
  double x1, x2, y1, y2,x11,x22,y11,y22;

  // Declaring variables
  // This is an actual image in matrix form
  Mat cameraFrame, detected_edges, blury, red_temp1, red_temp2, red;

  Rect2d bbox, bbox_big; 				  //Bounding boxes
  vector<Rect2d> act_bbox; 				  //Vector with bounding boxes
  vortex_msgs::CameraObjectInfo detected; //Message to be published



public:
    
    // Constructor runs run() function
     gateFinder(int argc, char** argv)
      : it_(nh_)
    {
      // Subscribe to input video feed and publish output video feed
      image_sub_ = it_.subscribe("/auv/auv/camerafront/camera_image", 1, &gateFinder::run, this);
      image_pub_ =it_.advertise("/camera/gate_detect",1);
      gate_pub_ = it_.advertise("/camera/gate_tuning",1);
      detect_pub_ = n_.advertise<vortex_msgs::CameraObjectInfo>("gate_midpoint",1000);
      cv::namedWindow(OPENCV_WINDOW);
    }
    ~gateFinder()
    {
      cv::destroyWindow(OPENCV_WINDOW);
    }
    
  // Dynamic tuning of color filter
  void configCallback(const gate_detect::GateParamsConfig &config, uint32_t level);

  // Setting message values to a default
  void init_msg(cv_bridge::CvImagePtr cv_ptr);

   // Red filter, blur and egde detection
  void redFilterAndEgde(cv_bridge::CvImagePtr cv_ptr);

  void Contours(cv_bridge::CvImagePtr cv_ptr); 
  
  // Displays windows on screen
  void drawOnImage(cv_bridge::CvImagePtr cv_ptr);

  void run(const sensor_msgs::ImageConstPtr& msg);
};



#endif
