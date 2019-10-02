#include "pole_detect_node.h"


// Dynamic tuning of color filter
void poleFinder::configCallback(const pole_detect::PoleParamsConfig &config, uint32_t level)
{
      //ROS_INFO_STREAM("config");
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
void poleFinder::init_msg(cv_bridge::CvImagePtr cv_ptr) 
{

  detected.header.stamp = ros::Time::now();
  detected.header.frame_id = CAMERA_FRAME;
  detected.frame_height = cv_ptr->image.rows;//bbox.height;
  detected.frame_width = cv_ptr->image.cols;//bbox.width;
  detected.confidence = 0;
  detected.pos_x = -1;
  detected.pos_y = -1;
}

 // Red filter, blur and egde detection
void poleFinder::redFilterAndEgde(cv_bridge::CvImagePtr cv_ptr) 
{
  // Converts an image from one color space to another.
  cvtColor(cv_ptr->image, hsv_image, CV_BGR2HSV);

  // hsv thresholds
  cv::Scalar lower_hsv_threshold_image1 = cv::Scalar(minhue1,minsat1,minval1);
  cv::Scalar upper_hsv_threshold_image1 = cv::Scalar(maxhue1,maxsat1,maxval1);
  cv::Scalar lower_hsv_threshold_image2 = cv::Scalar(minhue2,minsat2,minval2); 
  cv::Scalar upper_hsv_threshold_image2 = cv::Scalar(maxhue2,maxsat2,maxval2);

  // Threshold the HSV image, keep only the red pixels
  cv::inRange(hsv_image, lower_hsv_threshold_image1, upper_hsv_threshold_image1, lower_red_temp_image);
  cv::inRange(hsv_image, lower_hsv_threshold_image2, upper_hsv_threshold_image2, upper_red_temp_image);

  /* addWeighted calculates the weighted sum of two image arrays
     output array 'red' has the same size and number of channels as the input arrays 
	 site: https://docs.opencv.org/2.4/modules/core/doc/operations_on_arrays.html
  */
  
  alpha = 1.0;				// weight of the first array elements.
  beta = 1.0;				// weight of the second array elements.
  gamma = 0.0;				// scalar added to each sum

  cv::addWeighted(lower_red_temp_image, alpha, upper_red_temp_image, beta, gamma, red_image);

  // Image processing by applying Gaussian smoothing on the input source image
  // any sharp edges in images are smoothed while minimizing too much blurring.

  cv::Size KERNEL = cv::Size(9,9); // 9x9 averaging filter kernel
  sigmaX = sigmaY = 0;			   // Gaussian kernel standard deviation

  cv::GaussianBlur(red_image, blurred_image, KERNEL,sigmaX,sigmaY);

  /* Finds edges in an image using the [Canny86] algorithm
	 page: https://docs.opencv.org/3.3.1/da/d5c/tutorial_canny_detector.html
  */

  int lowThreshold = 10; // The value entered by the user moving the Trackbar
  int highThreshold = lowThreshold * 5; // Set in the program as three times the lower threshold (following Canny's recommendation)
  int kernel_size = 3; // We defined it to be 3 (the size of the Sobel kernel to be used internally)
  
  Canny(blurred_image, detected_edges, lowThreshold, highThreshold, kernel_size); 
}

void poleFinder::Contours(cv_bridge::CvImagePtr cv_ptr) 
{
  
  // Declaring necesarry variables  
  // Each contour is stored as a vector of points.
  vector<vector<Point> > contours;

  /* Finds contours in a binary image.
	 mode=CV_RETR_LIST:
	 	- retrieves all of the contours without establishing any hierarchical relationships
	 method=CV_CHAIN_APPROX_SIMPLE:
	 	- compresses horizontal, vertical, and diagonal segments and leaves only their 
	 	  end points. For example, an up-right rectangular contour is encoded with 4 points.
  */

  findContours(detected_edges, contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);

  // Filtering countours based on height into two vectors heights and heights2
  int height = 0; 
  for (int i = 0; i < contours.size(); i++)
  {
    bbox = boundingRect(contours[i]);
    
    if ( bbox.height > height ) {
      height = bbox.height;
      bbox_big = bbox;
    }
  }
}


void poleFinder::findDistance(cv_bridge::CvImagePtr cv_ptr) 
{
    detected.confidence = 0.5;
    bbox = bbox_big;
    detected.pos_x = (bbox.tl().x + bbox.br().x) / 2;
    detected.pos_y = (bbox.tl().y + bbox.br().y) / 2;
    rectangle(cv_ptr->image, bbox.tl(), bbox.br(), Scalar(0,255,0),5);

    /*
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
    } */
}


// Displays windows on screen
void poleFinder::drawOnImage(cv_bridge::CvImagePtr cv_ptr) 
{
    cv::imshow(OPENCV_WINDOW, red_image);
    cv::imshow(WINDOW2, cv_ptr->image);
	    cv::waitKey(3);
}
    
// opencb callback
void poleFinder::run(const sensor_msgs::ImageConstPtr& msg)
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
  
  // Dynamic reconfigure
  f = boost::bind(&poleFinder::configCallback, this, _1, _2);
  server.setCallback(f);


  //cv_ptr is the current image
  init_msg(cv_ptr);
  redFilterAndEgde(cv_ptr);
  Contours(cv_ptr);
  findDistance(cv_ptr);
  drawOnImage(cv_ptr);
  detect_pub_.publish(detected);
}


int main(int argc, char** argv)
{
  ros::init(argc, argv, "pole_detect");
  poleFinder ic(argc, argv);
  ros::spin();
  return 0;
}