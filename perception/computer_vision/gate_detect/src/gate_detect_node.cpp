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
#include <opencv2/tracking.hpp>
#include <opencv2/core/ocl.hpp>

#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <vector>
#include <algorithm> 
#include "std_msgs/Bool.h"
//#include "std_msgs/String.h"
#include <sstream>

using namespace cv;
using namespace std;


Mat convert_frame(cv_bridge::CvImagePtr frame);
vector<Vec4i> filter_lines(vector<Vec4i> lines);
vector<Vec4i> insertion_sort_lines(vector<Vec4i> lines);

vector<Vec4i> lines_filtered;
vector<Vec4i> lines_sorted;

//Video
Mat frame; 
Mat frame_converted;

static const std::string OPENCV_WINDOW = "Image window";

class ImageConverter
{
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;
  ros::Publisher detect_pub_;

public:
  ImageConverter()
    : it_(nh_)
  {
    // Subscrive to input video feed and publish output video feed
    image_sub_ = it_.subscribe("/camera/image", 1,
      &ImageConverter::imageCb, this);
    detect_pub_ = nh_.advertise<std_msgs::Bool>("detected_line", 1000);

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
		
		///////////////////////// CV ////////////////////////////

		/// Convert color of image
		frame_converted = convert_frame(cv_ptr);			

		// Probabilistic Line Transform
	  vector<Vec4i> lines; // will hold the results of the detection
	  HoughLinesP(frame_converted, lines, 1, CV_PI/180, 50, 50, 10 ); // runs the actual detection
		
		//Filter out lines that doesn't mach desired output
		lines_filtered = filter_lines(lines);
		
	    // Draw the lines
	  for( size_t i = 0; i < lines_filtered.size(); i++ )
	  {	
			Vec4i l = lines_filtered[i];
			line( frame_converted, Point(l[0], l[1]), Point(l[2], l[3]), Scalar(0,0,255), 2, LINE_AA);
	  }

		///////////////////////// END CV ////////////////////////////

		
    // Update GUI Window
		imshow(OPENCV_WINDOW, frame_converted);
    waitKey(3);

		///////////////////////// PUBLISH ////////////////////////////

    // Output modified video stream
		std_msgs::Bool detected;
		detected.data = false;

		if (lines_filtered.size() != 0)
			detected.data = true;

		ROS_INFO("%d", detected.data);
    detect_pub_.publish(detected);

		///////////////////////// END PUBLISH ////////////////////////////

  }
};






Mat convert_frame(cv_bridge::CvImagePtr frame) 
{
	Mat canny_output, kernel, frame_converted;

	cvtColor(frame->image, frame_converted, COLOR_BGR2HSV);
  GaussianBlur( frame_converted, frame_converted, Size(9,9), 0, 0);
	Canny( frame_converted, canny_output, 10, 50, 3 );
	kernel = Mat::ones(3, 3, CV_32F); //evt CV_8UC3
	dilate(canny_output, frame_converted, kernel);

	return frame_converted;
}


vector<Vec4i> filter_lines(vector<Vec4i> lines){
	vector<Vec4i> lines_filtered;
	int height, height_new, width, width_new, x_diff;
	vector<Vec4i>::iterator it_max;

	//iterate through every detected line
	for( size_t i = 0; i < lines.size(); i++ )
	{		
		Vec4i l_new = lines[i]; 

		//Notice: l[3] (y_1) may be smaller than l[1] (y_2)
		//l[2] > l[0]
		height_new = abs(l_new[3]-l_new[1]);
		width_new = abs(l_new[2]-l_new[0]);
		
		//dissmiss those that are not vertical
		if (width_new > 10)
		{
			continue; //Skip this iteration
		}		

		//make sure there are at least 1 object in the list to later compare
		if (lines_filtered.size() == 0)
			lines_filtered.insert(lines_filtered.begin(), l_new);

		//Iterate through the four highest lines
		vector<Vec4i>::iterator it = lines_filtered.begin();
		for (it; it < lines_filtered.end(); it++)
		{
			Vec4i l = *it;
			height = abs(l[3]-l[1]);

			//Find (if possible) position of highest line and insert 
			//before. Remove shortest
			if (height < height_new)
			{
				it_max = lines_filtered.insert(it, l_new);

				//Iterate through the highest lines, including the newly added
				vector<Vec4i>::iterator it_2 = lines_filtered.begin();
				for (it_2; it_2 < lines_filtered.end(); it_2++)
				{
					Vec4i l_2 = *it_2;
					x_diff = abs(l_new[0] - l_2[0]);
				
					//Check if "overlapping" lines
					if (x_diff < 10 && *it_max != l_2)
						lines_filtered.erase(it_2); 
				}
	
				break;				
			}

		}

		if (lines_filtered.size() > 4)
			lines_filtered.pop_back();				
		
	}

	return lines_filtered;
}


vector<Vec4i> insertion_sort_lines(vector<Vec4i> lines)
{
	vector<Vec4i> lines_sorted;
	Vec4i line, line_s;
	bool sort_flag = false;
	
	//check if empty
	if (lines.size() == 0)
	{	
		return lines;
	}

	//insert first element
	line_s = lines[0];
	lines_sorted.push_back(line_s);	

	//Iterate through the highest
	vector<Vec4i>::iterator it = lines.begin();
	it++;
	for (it; it < lines.end(); it++)
	{
		line = *it;
		sort_flag = false;

		// Iterate through sorted list
		vector<Vec4i>::iterator it_s = lines_sorted.begin();
		for (it_s; it_s < lines_sorted.end(); it_s++)
		{
			line_s = *it_s;
			//If already in list	
			if (line_s == line)
				continue; //Skip this iteration
			
			// If line (sorted by height) has a smaller x pos --> insert before
			if (line[0] < line_s[0])
			{
				lines_sorted.insert(it_s, line);
				sort_flag = true;
				break;
			}
							
		}

		if (!sort_flag)
			lines_sorted.push_back(line);

	}
	return lines_sorted;
}










int main(int argc, char** argv)
{
  ros::init(argc, argv, "image_converter");
  ImageConverter ic;
  ros::spin();




  return 0;
}


