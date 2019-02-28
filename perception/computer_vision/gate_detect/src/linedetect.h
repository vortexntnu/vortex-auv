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
vector<Vec4i> insertion_sort_lines(vector<Vec4i> lines);
vector<Vec4i> buffer_lines(vector<Vec4i> lines, vector<Vec4i> lines_buffered);

Rect2d lines2bbox(vector<Vec4i> lines);
bool tracker_init(int it_tracker, Mat frame, Rect2d bbox);
Mat tracker_update(Mat frame);



