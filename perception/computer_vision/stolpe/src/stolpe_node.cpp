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
#include <sstream>
// Quality of life
using namespace cv;
using namespace std;
using namespace cv_bridge;
using namespace image_transport;

static const std::string OPENCV_WINDOW = "Image window";
static const std::string WINDOW2 = "Image window 2";
class ImageConverter
{
  ros::NodeHandle nh_;
  ImageTransport it_;
  Subscriber image_sub_;
  ros::Publisher detect_pub_; 

  public:
    ImageConverter()
      : it_(nh_)
    {
      // Subscribe to input video feed and publish output video feed
      image_sub_ = it_.subscribe("/camera/image_raw",1,&ImageConverter::imageCb, this);
      detect_pub_ = nh_.advertise<std_msgs::Bool>("stolpe_detected",1000);
      // Image for publishing
      cv::namedWindow(OPENCV_WINDOW);
    }
    ~ImageConverter()
    {
      cv::destroyWindow(OPENCV_WINDOW);
    }

    
    void imageCb(const sensor_msgs::ImageConstPtr& msg)
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
    // Detect stople (Bør vel få klart for oss hva den skal detektere)
    // Declaring variables
    Mat cameraFrame, cameraFrameGrey, detected_edges, blury, red1, red2, red3;
    int height = 0;
    Rect2d bbox;
    vector<Rect2d> act_bbox;
    std_msgs::Bool detected;
    detected.data = false;
    // Reading stream and putting on a red mask
    cvtColor(cv_ptr->image, cameraFrameGrey, CV_BGR2HSV);
    inRange(cameraFrameGrey, Scalar(0,120,120), Scalar(20,255,255), red1);
    inRange(cameraFrameGrey, Scalar(160,120,120), Scalar(180,255,255), red2);
    // Combining red masks
    addWeighted(red1, 1.0, red2, 1.0, 0.0, red3);
    GaussianBlur(red3, blury, Size(9,9),0,0);
    //blur(cameraFrameGrey, blury, Size(9,9),Point(-1,-1));
    Canny(blury, detected_edges, 10, 50, 3);
    vector<vector<Point> > contours;
    findContours(detected_edges, contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);
    if (contours.size() > 0) {
    for(int i = 0; i < contours.size(); i++) {
      bbox = boundingRect(contours[i]);
      if (bbox.height > height) {
        height = bbox.height;
        act_bbox.push_back(bbox);  
      }
    }
    bbox = act_bbox.back();
    rectangle(cv_ptr->image, bbox.tl(), bbox.br(), Scalar(0,255,0),5);
       
    // Update GUI
    cv::imshow(OPENCV_WINDOW, red3);
    cv::imshow(WINDOW2, cv_ptr->image);
    cv::waitKey(3);
    // Output Bool value if detected or not
   
    if (bbox.area() > 2000) {
      detected.data = true;
    }
    }
    ROS_INFO("%d", detected.data);
    detect_pub_.publish(detected);
    
      
    }
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "image_converter");
  ImageConverter ic;
  ros::spin();
  return 0;
}



