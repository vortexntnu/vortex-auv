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
#include "CameraObjectInfo.h"
#include <sstream>
#include <ros/ros.h>
#include <dynamic_reconfigure/server.h>
#include <pole_detect/PoleParamsConfig.h>

// Quality of life
using namespace cv;
using namespace std;
using namespace cv_bridge;
using namespace image_transport;
static const std::string OPENCV_WINDOW = "Image window";
static const std::string WINDOW2 = "Image window 2";

class poleFinder
{
  // Setting up ROS handles, subsribers and publishers	
  ros::NodeHandle nh_;
  ros::NodeHandle n_;
  ImageTransport it_;
  Subscriber image_sub_;
  ros::Publisher detect_pub_;
  // Dynamic reconfigure
  dynamic_reconfigure::Server<pole_detect::PoleParamsConfig> server;
  dynamic_reconfigure::Server<pole_detect::PoleParamsConfig>::CallbackType f;
  // Declaring variables
  Mat cameraFrame, detected_edges, blury, red_temp1, red_temp2, red; //frames
  double x1, x2, y1, y2,x11,x22,y11,y22; //cordinates
  Rect2d bbox, bbox_big; //Bounding boxes
  vector<Rect2d> act_bbox; //Vector with bounding boxes
  pole_detect::CameraObjectInfo detected; //Message to be published
  int minhue1,maxhue1,minval1,maxval1,minsat1,maxsat1,minhue2,maxhue2,minval2,maxval2,minsat2,maxsat2;
  float distance;
  //-------------- HAVE TO BE TUNED ---------
  float width_pole = 0.4; // Height pixels of an object 1m from camera
  float focal_length = 332.5; // F = (PxD) / W (P - Pixle width, D - Distance, W - width of pole)
  
  public:



     // Constructor runs run() function
    poleFinder(int argc, char** argv)
      : it_(nh_)
    {
      // Subscribe to input video feed and publish output video feed
      image_sub_ = it_.subscribe("/image", 1, &poleFinder::run, this);
      detect_pub_ = n_.advertise<pole_detect::CameraObjectInfo>("pole_midpoint",1000);
      cv::namedWindow(OPENCV_WINDOW);
    }
    ~poleFinder()
    {
      cv::destroyWindow(OPENCV_WINDOW);
    }

    // Dynamic tuning of color filter
    void configCallback(const pole_detect::PoleParamsConfig &config, uint32_t level){
        ROS_INFO_STREAM("Info");
          minhue1 = config.minhue1;
          maxhue1 = config.maxhue1;
          minval1 = config.minval1;
          maxval1 = config.maxval1;
          minsat1 = config.minsat1;
          maxsat1 = config.maxsat1;
          minhue2 = config.minhue2;
          maxhue2 = config.maxhue2;
          minval2 = config.minval2;
          maxval2 = config.maxval2;
          minsat2 = config.minsat2;
          maxsat2 = config.maxsat2;
    }
    // Setting message values to a default
    void init_msg(cv_bridge::CvImagePtr cv_ptr) {
      detected.frame_height = cv_ptr->image.rows;//bbox.height;
      detected.frame_width = cv_ptr->image.cols;//bbox.width;
      detected.confidence = 0;
      detected.pos_x = -1;
      detected.pos_y = -1;
    }

     // Red filter, blur and egde detection
    void redFilterAndEgde(cv_bridge::CvImagePtr cv_ptr) {
      cvtColor(cv_ptr->image, cameraFrame, CV_BGR2HSV);
      inRange(cameraFrame, Scalar(minhue1,minsat1,minval1), Scalar(maxhue1,maxsat1,maxval1), red_temp1);
      inRange(cameraFrame, Scalar(minhue2,minsat2,minval2), Scalar(maxhue2,maxsat2,maxval2), red_temp2);
      addWeighted(red_temp1, 1.0, red_temp2, 1.0, 0.0, red);
      GaussianBlur(red, blury, Size(9,9),0,0);
      Canny(blury, detected_edges, 10, 50, 3);
    }

    void Contours(cv_bridge::CvImagePtr cv_ptr) {
      // Declearing necesarry variables  
      vector<vector<Point> > contours;
      findContours(detected_edges, contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);
      int height = 0; 

       // Filtering countours based on height into two vectors heights and heights2
      for (int i = 0; i < contours.size(); i++) {
        bbox = boundingRect(contours[i]);
        if ( bbox.height > height ) {
          height = bbox.height;
          bbox_big = bbox;
        }
      }
    }

    void findDistance(cv_bridge::CvImagePtr cv_ptr) {
        detected.confidence = 0.5;
        bbox = bbox_big;
        detected.pos_x = (bbox.tl().x + bbox.br().x) / 2;
        detected.pos_y = (bbox.tl().y + bbox.br().y) / 2;
        rectangle(cv_ptr->image, bbox.tl(), bbox.br(), Scalar(0,255,0),5);
        if (detected.pos_x > (detected.frame_width/2 - 200) && detected.pos_x < (detected.frame_width/2 + 200)) {
            if  (detected.pos_y > (detected.frame_height/2 - 200) && detected.pos_y < (detected.frame_height/2 + 200)) {
                bbox = bbox_big;
                distance = (focal_length *  0.4) / (float)bbox.width;
                detected.distance_to_pole = distance;

                distance = round(distance*100) / 100.0;
                std::string str = std::to_string(distance);
                str.erase(str.length()-4,4);
                str += "m";
                cv::Point point(10,60);
                cv::putText(cv_ptr->image, str, point, 3, 2,(0,255,255),1,1);
            }  
        }
    }

    // Displays windows on screen
    void drawOnImage(cv_bridge::CvImagePtr cv_ptr) {
        cv::imshow(OPENCV_WINDOW, red);
        cv::imshow(WINDOW2, cv_ptr->image);
   	    cv::waitKey(3);
    }
    
    void run(const sensor_msgs::ImageConstPtr& msg)
    // Reading the image to cv_ptr
    {
      cv_bridge::CvImagePtr cv_ptr;  
      try
      {
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
      }
      catch(cv_bridge::Exception& e)
      {
        ROS_ERROR("cv_bridge exceptions: %s", e.what());
        return;
      }
      
      //cv_ptr is the current image
      init_msg(cv_ptr);
      redFilterAndEgde(cv_ptr);
      Contours(cv_ptr);
      findDistance(cv_ptr);
      drawOnImage(cv_ptr);
      f = boost::bind(&poleFinder::configCallback, this, _1, _2);
      server.setCallback(f);
      detect_pub_.publish(detected);
    }

};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "pole_detect");
  poleFinder ic(argc, argv);
  ros::spin();
  return 0;
}