#include "gate_detect_node.h"

/*   Written by Ambjørn Grimsrud Waldum, Student
     Edited by Kristoffer Rakstad Solberg, Student
     Copyright (c) 2020 Manta AUV, Vortex NTNU.
     All rights reserved. */

void gateFinder::configCallback(const gate_detect::GateParamsConfig &config, uint32_t level)
{
       
    minhue = config.minhue;
    maxhue = config.maxhue;
    minval = config.minval;
    minsat = config.minsat;
    maxval = config.maxval;
    maxsat = config.maxsat;
    height = config.pole_height_treshold;
}

// Setting message values to a default
void gateFinder::init_msg(cv_bridge::CvImagePtr cv_ptr) 
{
    detected.frame_height = cv_ptr->image.rows;//bbox.height;
    detected.frame_width = cv_ptr->image.cols;//bbox.width;
    detected.confidence = 0;
    detected.pos_x = -1;
    detected.pos_y = -1;
    detected.poles_leaving_image = 0;
    }

void gateFinder::redFilterAndEgde(cv_bridge::CvImagePtr cv_ptr) 
{
     cvtColor(cv_ptr->image, cameraFrame, CV_BGR2HSV);
     inRange(cameraFrame, Scalar(minhue,minsat,minval), Scalar(maxhue,maxsat,maxval), red);
     GaussianBlur(red, blury, Size(9,9),0,0);
     Canny(blury, detected_edges, 10, 50, 3);
}

void gateFinder::Contours(cv_bridge::CvImagePtr cv_ptr)
{
      
  // Declearing necesarry variables  
  vector<vector<Point> > contours;
  vector<Rect2d> heights;
  vector<Rect2d> heights2;

  
  findContours(detected_edges, contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);

   // Filtering countours based on height into two vectors heights and heights2
  for (int i = 0; i < contours.size(); i++) 
  {
     bbox = boundingRect(contours[i]);
     if ( bbox.height > height && bbox.width < 5.0*height)
     {
       height = bbox.height;
       heights.push_back(bbox);
     }
  }

  if (heights.size() > 0) 
  {
    bbox_big = heights.end()[-1];
    int center = (bbox_big.tl().x + bbox_big.br().x)/2;
    height = 0;

    for (int i = 0; i < contours.size(); i++) 
    {
      bbox = boundingRect(contours[i]);
      int center2 = (bbox.tl().x + bbox.br().x)/2;
      if ((center2 > center + 25 || center2 < center - 25) && !bbox_big.contains(bbox.tl()) && !bbox_big.contains(bbox.br())) 
      {
        if (bbox.height > height) 
        {
          heights2.push_back(bbox);
        }
      } 
    }
  }
        // If two contours are found, that are not on top of each other, update message with information
  if (heights.size() > 0 && heights2.size() > 0) 
  {
      bbox = heights.end()[-1];
      rectangle(cv_ptr->image, bbox.tl(), bbox.br(), Scalar(255,0,0),5);
      x1 = bbox.tl().x;
      y1 = bbox.tl().y;
      x2 = bbox.br().x;
      y2 = bbox.br().y;
      bbox = heights2.end()[-1];
      x11 = bbox.tl().x;
      y11 = bbox.tl().y;
      x22 = bbox.br().x;
      y22 = bbox.br().y;
      rectangle(cv_ptr->image, bbox.tl(), bbox.br(), Scalar(255,0,0),5);
      detected.confidence = 1.0;
      detected.pos_x = (x11+x22)/2 + (((x1+x2)/2 - (x11+x22)/2)/2);
      detected.pos_y = ((y11+y22)/2 + (((y1+y2)/2 - (y11-y22)/2)/2))+50;  


        if (x1 < detected.frame_width * 0.1 || x2 > detected.frame_width - detected.frame_width * 0.1 || x11 < detected.frame_width * 0.1 || x22 > detected.frame_width - detected.frame_width * 0.1) 
        {
            detected.confidence = 0.5;
        }

        if (detected.confidence == 1)
        {
            if ( ((x1+x2)/2 < detected.frame_width*0.1 || (x1+x2)/2 > detected.frame_width*0.9) && ((x11+x22)/2 < detected.frame_width*0.1 || (x11+x22)/2 > detected.frame_width*0.9)) {
            detected.poles_leaving_image = 1;
        }
      }
  }

}

// Displays windows on screen
void gateFinder::drawOnImage(cv_bridge::CvImagePtr cv_ptr) 
{
    cv::imshow(OPENCV_WINDOW, red);
    cv::imshow(WINDOW2, cv_ptr->image);
    cv::waitKey(3);
}


void gateFinder::run(const sensor_msgs::ImageConstPtr& msg)
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
  f = boost::bind(&gateFinder::configCallback, this, _1, _2);
  server.setCallback(f);



  //cv_ptr is the current image
  init_msg(cv_ptr);
  redFilterAndEgde(cv_ptr);
  Contours(cv_ptr);
  drawOnImage(cv_ptr);
  detect_pub_.publish(detected);
  image_pub_.publish(cv_ptr->toImageMsg());
  sensor_msgs::ImagePtr image = cv_bridge::CvImage(std_msgs::Header(), "mono8", red).toImageMsg();
  gate_pub_.publish(image);
 
}


/* MAIN */
int main(int argc, char** argv)
{
  ros::init(argc, argv, "gate_detect");
  gateFinder ic(argc, argv);
  ros::spin();
  return 0;
}
