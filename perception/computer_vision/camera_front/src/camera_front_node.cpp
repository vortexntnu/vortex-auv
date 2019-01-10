//
//  main.cpp
//  CV
//
//  Created by Thomas Hellum on 22/10/2018.
//  Copyright © 2018 Thomas Hellum. All rights reserved.
//

#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include <opencv2/opencv.hpp>
#include <opencv2/tracking.hpp>
#include <opencv2/core/ocl.hpp>


#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <vector>
#include <algorithm> 


using namespace cv;
using namespace std;

/*
class CameraFront
{
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;
  image_transport::Publisher image_pub_;

public:
	CameraFront()
		: it_(nh_)
	{

		vector<Vec4i> lines_filtered;
		vector<Vec4i> lines_sorted;
	
		//Video
		Mat frame; 
		Mat frame_converted;

		// Subscrive to input video feed and publish output video feed
		image_sub_ = it_.subscribe("/camera/CameraFront", 1,
		&CameraFront::imageCb, this);
		image_pub_ = it_.advertise("/image_converter/output_video", 1);

		cv::namedWindow(OPENCV_WINDOW, WINDOW_NORMAL);
		cv::namedWindow(OPENCV_WINDOW, WINDOW_NORMAL);
	}

	~CameraFront()
	{
		cv::destroyWindow(OPENCV_WINDOW);
		cv::destroyWindow(OPENCV_WINDOW2);
	}

	Mat blur(cv_brigde::CvImagePtr frame)
	{
		Mat frame_mat;
		GaussianBlur(frame->image, frame_mat, Size(3,3), 0, 0);
		return frame_mat;
	}

  void imageCb(const sensor_msgs::ImageConstPtr& msg)
  {
    cv_bridge::CvImagePtr cv_ptr;
		Mat frame;
		vector<Vec4i> lines;
    try
    {	
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }

		frame = blur(cv_ptr);

    // Update GUI Window
    //cv::imshow(OPENCV_WINDOW, frame);
    //

		cv::imshow(OPENCV_WINDOW2, cv_ptr->image);
		cv::waitKey(3);
    // Output modified video stream
    image_pub_.publish(cv_ptr->toImageMsg());
  }
};

class CameraFront
{
	sensor_msgs::ImagePtr toImageMsg() const;
  // Overload mainly intended for aggregate messages that contain
  // a sensor_msgs::Image as a member.
  void toImageMsg(sensor_msgs::Image& ros_image) const;
};
*/

int main() {
	/*
	ros::init(argc, argv, "image_converter");
	CameraFront ic;
	ros::spin();
	*/
	
	//Video
	Mat frame; 
  const char* gst = "/home/hellum/Videos/GOPR1142.avi";
	VideoCapture video(gst); //capture the video from video, change to "1" for cam
    
	// if not success, exit program
	if ( !video.isOpened() )  
	{
		cout << "Cannot open the web cam" << endl;
		return -1;
	}	
	
	while(video.read(frame))
	{	
		/// Show in a window		
		namedWindow( "Display", CV_WINDOW_NORMAL );
		resizeWindow( "Display", 1260, 1080);	  	
		imshow( "Display", frame );
		
	
		if(waitKey(1)==27)
			break;
	}	
	return 0;	

}		