

#ifndef POLE_DETECT_NODE_H
#define POLE_DETECT_NODE_H

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
//#include "CameraObjectInfo.h"
#include "vortex_msgs/CameraObjectInfo.h"
#include <sstream>
#include <ros/ros.h>
#include <dynamic_reconfigure/server.h>
#include <pole_detect/PoleParamsConfig.h>

// Quality of life
using namespace cv;
using namespace std;
using namespace cv_bridge;
using namespace image_transport; 	
static const std::string OPENCV_WINDOW = "Binary image, pole";
static const std::string WINDOW2 = "Bounding boxes, pole";
static const std::string CAMERA_FRAME = "auv/camerafront_link";


class poleFinder{

private:

	 // Setting up ROS handles, subsribers and publishers	
	 ros::NodeHandle nh_;
	 ros::NodeHandle n_;
	 ImageTransport it_;

	 // ros topics
	 Subscriber image_sub_;
	 Publisher image_pub_;
	 Publisher red_image_pub_;
	 ros::Publisher detect_pub_;
	  
	 // Dynamic reconfigure
	 dynamic_reconfigure::Server<pole_detect::PoleParamsConfig> server;
	 dynamic_reconfigure::Server<pole_detect::PoleParamsConfig>::CallbackType f;
	  
	 // hsv variables
	 int minhue,maxhue,minval,maxval,minsat,maxsat;

	 // Pole height treshold for detection
	 double height;

	 // image weights
	 double alpha, beta, gamma;

	 // Gaussian kernel standard deviation
	 double sigmaX, sigmaY;

	 // object distance
	 float distance;

	 // image frame coordinates
	 double x1, x2, y1, y2,x11,x22,y11,y22;

	 // cv::Mat - n-dimensional dense array class 
	 // This is an actual image in matrix form
	 cv::Mat hsv_image, detected_edges, blurred_image, lower_red_temp_image, upper_red_temp_image, red_image;


	 cv::Rect2d bbox, bbox_big; 			 //Bounding boxes
	 vector<Rect2d> act_bbox; 				 //Vector with bounding boxes
	 vortex_msgs::CameraObjectInfo detected; //Message to be published

	  
	 //-------------- HAVE TO BE TUNED --------------
	 float width_pole = 0.4; // Height pixels of an object 1m from camera
	 float focal_length = 332.5; // F = (PxD) / W (P - Pixle width, D - Distance, W - width of pole)


public:

    // Constructor runs run() function
    poleFinder(int argc, char** argv)
      : it_(nh_)
    {
      // Subscribe to input video feed and publish output video feed
      image_sub_ = it_.subscribe("/auv/auv/camerafront/camera_image", 1, &poleFinder::run, this);
	  image_pub_ = it_.advertise("/camera/pole_detect",1);
	  red_image_pub_ = it_.advertise("/camera/pole_tuning",1);
      detect_pub_ = n_.advertise<vortex_msgs::CameraObjectInfo>("pole_midpoint",1000);
      cv::namedWindow(OPENCV_WINDOW);
    }

    // Destructor
    ~poleFinder(){
      cv::destroyWindow(OPENCV_WINDOW);
    }


    /**** FUNCTIONS  ****/

    // dynamic reconfigure
    void configCallback(const pole_detect::PoleParamsConfig &config, uint32_t level);

    // CvImagePtr is a shared ptr of the CvImage class that is interoperable with
    // sensor_msgs/Image, but uses a more convenient cv::Mat representation for the
    // Image data
    void init_msg(cv_bridge::CvImagePtr cv_ptr);

    // red filtering and edge detection in image
    void redFilterAndEgde(cv_bridge::CvImagePtr cv_ptr);

    // finds contours in a binary image
    void Contours(cv_bridge::CvImagePtr cv_ptr);

    
    void findDistance(cv_bridge::CvImagePtr cv_ptr);

    void drawOnImage(cv_bridge::CvImagePtr cv_ptr);

    void run(const sensor_msgs::ImageConstPtr& msg);

};

#endif
