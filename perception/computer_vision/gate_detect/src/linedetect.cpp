//
//  main.cpp
//  CV
//
//  Created by Thomas Hellum on 22/10/2018.
//  Copyright © 2018 Thomas Hellum. All rights reserved.
//

/*



#include <opencv2/highgui.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/imgproc.hpp>
#include "opencv2/imgcodecs.hpp"
*/


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

Mat convert_frame(Mat input_frame);
vector<Vec4i> filter_lines(vector<Vec4i> lines);
bool tracker_init(int it_tracker, Mat frame, Rect2d bbox);
Mat tracker_update(Mat frame);
vector<Vec4i> insertion_sort_lines(vector<Vec4i> lines);
Rect2d lines2bbox(vector<Vec4i> lines);


// create a tracker object
Ptr<Tracker> tracker;

// Creat a boundingbox
Rect2d bbox;

// Detection flag for tracking
bool detection_flag = false;
bool tracker_initiated = false;


int main( int argc, char** argv ) {
	//lines
	vector<Vec4i> lines_filtered;
	vector<Vec4i> lines_sorted;
	
	//Video
	Mat frame; 
	Mat frame_converted;
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
		// Start timer
        double timer = (double)getTickCount();


		/////// DETECT LINES ///////

		if (!detection_flag)
		{ 
			/// Convert color of image
			frame_converted = convert_frame(frame);			

			// Probabilistic Line Transform
		    	vector<Vec4i> lines; // will hold the results of the detection
		    	HoughLinesP(frame_converted, lines, 1, CV_PI/180, 50, 50, 10 ); // runs the actual detection
			//Filter out lines that doesn't mach desired output
			lines_filtered = filter_lines(lines);
	
		    	// Draw the lines
		    	for( size_t i = 0; i < lines_filtered.size(); i++ )
		    	{	
				cout << lines_filtered[0][0]  << endl;
				Vec4i l = lines_filtered[i];
				line( frame, Point(l[0], l[1]), Point(l[2], l[3]), Scalar(0,0,255), 2, LINE_AA);
		    	}
	    	}
		/////// TRACK ///////
		
		if (lines_filtered.size() == 4)
		{
			detection_flag = true;
			
			// Sort by x_pos
			lines_sorted = insertion_sort_lines(lines_filtered);		
		
			bbox = lines2bbox(lines_sorted);
			
			if (!tracker_initiated)
				tracker_initiated = tracker_init(4, frame, bbox);
	
			frame = tracker_update(frame);
		}

		/////// COMMON TASKS ///////
		
		// Calculate Frames per second (FPS)
        	float fps = getTickFrequency() / ((double)getTickCount() - timer);
		
		// Display FPS on frame
		cout << "FPS: " << fps << endl;	        	
	  	
		/// Show in a window		
		namedWindow( "Display", CV_WINDOW_NORMAL );
		resizeWindow( "Display", 1260, 1080);	  	
		imshow( "Display", frame );
		
	
		if(waitKey(1)==27)
			break;
	}

    return 0;
}



Mat convert_frame(Mat frame) 
{
	Mat canny_output, kernel, frame_converted;

	cvtColor(frame, frame_converted, COLOR_BGR2HSV);
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


Rect2d lines2bbox(vector<Vec4i> lines)
{
	//Find top corner
	int x_0 = lines[0][0];	
	int y_0 = max(lines[0][3], lines[0][1]);	

	//Find width and height
	int x_1 = lines[4][0];
	int width = abs(x_1 - x_0);

	int y_1 = min(lines[0][3], lines[0][1]);
	int height = abs(y_1 - y_0);
	
	//Create bounding box
	Rect2d bounding_box;
	bounding_box.x = x_0;
	bounding_box.y = y_0;
	bounding_box.width = width;
	bounding_box.height = height;

	return bbox;
}




bool tracker_init(int it_tracker, Mat frame, Rect2d bbox){

	//choose tracker type
	string trackerTypes[6] = {"BOOSTING", "MIL", "KCF", "TLD","MEDIANFLOW", "GOTURN"};
	string trackerType = trackerTypes[it_tracker];

  	//Ptr<Tracker> tracker = TrackerKCF::create();
	if (trackerType == "BOOSTING")
      		tracker = TrackerBoosting::create();
	if (trackerType == "MIL")
	      	tracker = TrackerMIL::create();
	if (trackerType == "KCF")
	      	tracker = TrackerKCF::create();
	if (trackerType == "TLD")
	      	tracker = TrackerTLD::create();
	if (trackerType == "MEDIANFLOW")
	      	tracker = TrackerMedianFlow::create();
	if (trackerType == "GOTURN")
	      	tracker = TrackerGOTURN::create();

	tracker->init(frame, bbox);
	rectangle(frame, bbox, Scalar( 255, 0, 0 ), 2, 1 );
	return true; 
}
 
Mat tracker_update(Mat frame)
{ 
        // Update the tracking result
        bool ok = tracker->update(frame, bbox);
         
        if (ok)
        {
            // Tracking success : Draw the tracked object
            rectangle(frame, bbox, Scalar( 255, 0, 0 ), 2, 1 );
        }
        else
        {
            // Tracking failure detected.
            cout << "Tracking failure detected" << endl;
        }

	return frame;
}
 

vector<Vec4i> buffer_lines(vector<Vec4i> lines, vector<Vec4i> lines_buffered){
	vector<Vec4i> lines_sorted;	
	Vec4i line, line_buf;	
	
	//check if empty
	if (lines_buffered.size() < 4)
	{	
		return lines;
	}
	
	if (lines_buffered.size() != lines.size())
	{	
		return lines_buffered;
	}
	
	vector<Vec4i>::iterator it = lines.begin();
	vector<Vec4i>::iterator it_buf = lines_buffered.begin();
	int count = 0;
	for (it; it < lines.end(); it++)
	{
		line = *it;
		line_buf = *it_buf;

		if (abs(line[1] - line[3]) > abs(line_buf[1] - line_buf[3]))
		{
			/*			
			cout << "Before:" << endl;
			vector<Vec4i>::iterator it_buf2 = lines_buffered.begin();
			for (it_buf2; it_buf2 < lines.end(); it_buf2++)
			{
				line = *it_buf2;
				cout << line[0] << endl;
			}
			*/

			lines_buffered.erase(lines_buffered.begin()+count);
			it_buf = lines_buffered.insert(it_buf, line);

			/*
			cout << "After:" << endl;
			it_buf2 = lines_buffered.begin();
			for (it_buf2; it_buf2 < lines.end(); it_buf2++)
			{
				line = *it_buf2;
				cout << line[0] << endl;
			}
			*/
		}
		it_buf++;
		count++;	
	}	
		
	return lines_buffered;
}



