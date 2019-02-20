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
#include "stolpe/CameraObjectInfo.h"
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
  ros::NodeHandle n_;
  ImageTransport it_;
  Subscriber image_sub_;
  ros::Publisher detect_pub_; 

  public:
    ImageConverter()
      : it_(nh_)
    {
      // Subscribe to input video feed and publish output video feed
      image_sub_ = it_.subscribe("/manta/manta/camera/camera_image",1,&ImageConverter::imageCb, this);
      detect_pub_ = n_.advertise<stolpe::CameraObjectInfo>("/camera_object_info",1000);
      //detect_pub_ = nh_.advertise<std_msgs::Bool>("stolpe_detected",1000);
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
    double x1, x2, y1, y2,x11,x22,y11,y22;
    Rect2d bbox;
    Rect2d bbox_big;
    vector<Rect2d> act_bbox;
    stolpe::CameraObjectInfo detected_2;
    detected_2.frame_height = cv_ptr->image.rows;//bbox.height;
    detected_2.frame_width = cv_ptr->image.cols;//bbox.width;
    detected_2.confidence = 0;
    detected_2.pos_x = -1;
    detected_2.pos_y = -1;	
    //std_msgs::Bool detected;
    //detected.data = false;		

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
		vector<Rect2d> heights;
		vector<Rect2d> heights2;
    		findContours(detected_edges, contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);
		int height = 0;
		
		for (int i = 0; i < contours.size(); i++) {
			bbox = boundingRect(contours[i]);
			if ( bbox.height > height ) {
				height = bbox.height;
				heights.push_back(bbox);
			}
		
		}
		
		if (heights.size() > 0) {
			bbox_big = heights.end()[-1];
			int center = (bbox_big.tl().x + bbox_big.br().x)/2;
			height = 0;
		
			for (int i = 0; i < contours.size(); i++) {
				bbox = boundingRect(contours[i]);
				int center2 = (bbox.tl().x + bbox.br().x)/2;
				if ((center2 > center + 25 || center2 < center - 25) && !bbox_big.contains(bbox.tl()) && !bbox_big.contains(bbox.br())) {
					if (bbox.height > height) {
						heights2.push_back(bbox);
					}
				} 
			}
		}

	
		if (heights.size() > 0 && heights2.size() > 0) {
			bbox = heights.end()[-1];
			rectangle(cv_ptr->image, bbox.tl(), bbox.br(), Scalar(255,0,0),5);
			x1 = bbox.tl().x;
      			y1 = bbox.tl().y;
      			x2 = bbox.br().x;
      			y2 = bbox.br().y;
			std::cout << "x1: " << x1 << std::endl;
			bbox = heights2.end()[-1];
			x11 = bbox.tl().x;
      			y11 = bbox.tl().y;
      			x22 = bbox.br().x;
      			y22 = bbox.br().y;
			std::cout << "x11: " << x11 << std::endl;
			rectangle(cv_ptr->image, bbox.tl(), bbox.br(), Scalar(0,255,0),5);
    			detected_2.confidence = 1;
  	 		detected_2.pos_x = (x11+x22)/2 + (((x1+x2)/2 - (x11+x22)/2)/2);
    			detected_2.pos_y = ((y11+y22)/2 + (((y1+y2)/2 - (y11-y22)/2)/2))+50;	
			detected_2.confidence = 1;
		}
		cv::imshow(OPENCV_WINDOW, red3);
    		cv::imshow(WINDOW2, cv_ptr->image);
   		cv::waitKey(3);
		detect_pub_.publish(detected_2);
	}

	
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "gate_detect");
  ImageConverter ic;
  ros::spin();
  return 0;
}



