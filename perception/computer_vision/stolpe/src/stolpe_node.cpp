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
// Quality of life
using namespace cv;
using namespace std;
using namespace cv_bridge;
using namespace image_transport;

static const std::string OPENCV_WINDOW = "Image window";

class ImageConverter
{
  ros::NodeHandle nh_;
  ImageTransport it_;
  Subscriber image_sub_;
  Publisher image_pub_;

  public:
    ImageConverter()
      : it_(nh_)
    {
      // Subscribe to input video feed and publish output video feed
      image_sub_ = it_.subscribe("/camera/image_raw",1,&ImageConverter::imageCb, this);
      image_pub_ = it_.advertise("/image_converter/output_video",1);
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
    int height = 0;
    int count = 0;
    vector<Rect2d> heights;
    Rect2d* buffer = new Rect2d[10];  
    Mat cameraFrame, cameraFrameGrey, detected_edges, blury, red1, red2, red3;
    // Reading stream and putting on a red mask
    cvtColor(cv_ptr->image, cameraFrameGrey, CV_BGR2HSV);
    inRange(cameraFrameGrey, Scalar(0,0,50), Scalar(20,255,255), red1);
    inRange(cameraFrameGrey, Scalar(160,0,60), Scalar(180,255,255), red2);
    // Combining red masks
    addWeighted(red1, 1.0, red2, 1.0, 0.0, red3);
    GaussianBlur(red3, blury, Size(9,9),0,0);
    //blur(cameraFrameGrey, blury, Size(9,9),Point(-1,-1));
    Canny(blury, detected_edges, 10, 50, 3);
    vector<vector<Point> > contours;
    findContours(detected_edges, contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);
    Rect2d bbox = boundingRect(contours[0]);  
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
      rectangle(cv_ptr->image, bbox.tl(), bbox.br(), Scalar(0,255,0),5);
    }   
    // Update GUI
    cv::imshow(OPENCV_WINDOW, red3);
    cv::waitKey(3);
    // Output modified video stream
    image_pub_.publish(cv_ptr->toImageMsg());  
    }
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "image_converter");
  ImageConverter ic;
  ros::spin();
  return 0;
}




/*int want(int arg) {
  if (arg == 1) {
    return 0;
  }
 return 1;
}

int main(int argc, char** arg)
  {
  VideoCapture stream1("./test.m4v"); 

  if (!stream1.isOpened()) {
    cout <<"Cannot open camera";
  }
  int height = 0;
  int count = 0;
  vector<Rect2d> heights;
  Rect2d* buffer = new Rect2d[10];


  while (true) {
    Mat cameraFrame, cameraFrameGrey, detected_edges, blury, red1, red2, red3;

    // Reading stream and putting on a red mask
    stream1.read(cameraFrame);
    cvtColor(cameraFrame, cameraFrameGrey, CV_BGR2HSV);
    inRange(cameraFrameGrey, Scalar(0,0,50), Scalar(20,255,255), red1);
    inRange(cameraFrameGrey, Scalar(160,0,60), Scalar(180,255,255), red2);
    // Combining red masks
    addWeighted(red1, 1.0, red2, 1.0, 0.0, red3);
    GaussianBlur(red3, blury, Size(9,9),0,0);
    //blur(cameraFrameGrey, blury, Size(9,9),Point(-1,-1));
    Canny(blury, detected_edges, 10, 50, 3);
    vector<vector<Point> > contours;
    vector<Vec4i> hierarchy;
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
      rectangle(cameraFrame, bbox.tl(), bbox.br(), Scalar(0,255,0),5);
    }  
    imshow("Display", cameraFrame);
    if(waitKey(30) >= 0) {
      return 0;
    }

  }
  destroyWindow("Display");
  delete [] buffer;
}
*/

