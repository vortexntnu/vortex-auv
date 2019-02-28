//
//  main.cpp
//  CV
//
//  Created by Thomas Hellum on 22/10/2018.
//  Copyright © 2018 Thomas Hellum. All rights reserved.
//

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <opencv2/opencv.hpp>
#include <iostream>
#include <list>
#include <vector>

using namespace cv;
using namespace std;
/*
Mat CvImagePtr_to_Mat(cv_bridge::CvImagePtr frame);
Mat convert_frame(cv_bridge::CvImagePtr frame);

int height = 0;
int count = 0;
vector<Rect2d> heights;
Rect2d* buffer = new Rect2d[10];

//Video
Mat frame; 
Mat frame_converted;

static const std::string OPENCV_WINDOW = "Image window";

class ImageConverter
{
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;
  image_transport::Publisher image_pub_;

public:
  ImageConverter()
    : it_(nh_)
  {
    // Subscrive to input video feed and publish output video feed
    image_sub_ = it_.subscribe("/camera/image", 1,
      &ImageConverter::imageCb, this);
    image_pub_ = it_.advertise("/image_converter/output_video", 1);

    namedWindow(OPENCV_WINDOW);
  }

  ~ImageConverter()
  {
    destroyWindow(OPENCV_WINDOW);
  }

  void imageCb(const sensor_msgs::ImageConstPtr& msg)
  {
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }
		
		///////////////////////// CODE ////////////////////////////

		Mat detected_edges, contours, blurred;

		/// Convert color of image
		blurred = CvImagePtr_to_Mat(cv_ptr);
		detected_edges = convert_frame(cv_ptr);			

		vector<vector<Point> > contours;
    findContours(detected_edges, contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);
    Rect2d bbox = boundingRect(contours[0]);

		int height = 0;
    if (count == 0) {
      for(int i = 0; i < contours.size(); i++) {
     		Rect2d bbox = boundingRect(contours[i]);
     		if( bbox.height > height) {
       		height = bbox.height;
       		heights.push_back(bbox); 
     		}     
      }
    }
    count++;
    if (count == 4) {
      count = 0;
    }
   
    if (heights.size() > 0) {
      bbox = heights.back();
      rectangle(blurred, bbox.tl(), bbox.br(), Scalar(0,255,0),5);
    }

		///////////////////////// END CODE ////////////////////////////

		
    // Update GUI Window
		imshow(OPENCV_WINDOW, frame_converted);
    waitKey(3);

    // Output modified video stream
    image_pub_.publish(cv_ptr->toImageMsg());
  }
};

Mat CvImagePtr_to_Mat(cv_bridge::CvImagePtr frame) 
{
  GaussianBlur(frame->image, frame_converted, Size(9,9),0,0);
}


Mat convert_frame(cv_bridge::CvImagePtr frame) 
{
	Mat frame_converted, blurry, red1, red2, red3;

	cvtColor(frame->image, frame_converted, CV_BGR2HSV);
  inRange(frame_converted, Scalar(0,0,50), Scalar(20,255,255), red1);
  inRange(frame_converted, Scalar(160,0,60), Scalar(180,255,255), red2);
  // Combining red masks
  addWeighted(red1, 1.0, red2, 1.0, 0.0, red3);
  GaussianBlur(red3, blurry, Size(9,9),0,0);
  //blur(cameraFrameGrey, blury, Size(9,9),Point(-1,-1));
  Canny(blury, frame_converted, 10, 50, 3);

	return frame_converted;
}






*/

int main(int argc, char** argv)
{
  /*
	ros::init(argc, argv, "image_converter");
  ImageConverter ic;
  ros::spin();
	*/
  return 0;
}

