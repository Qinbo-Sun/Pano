#pragma once

#pragma warning(disable:4996)

//For sixCameraStitching.h
#include "opencv2/opencv.hpp"
#include "opencv2/stitching/detail/blenders.hpp"
#include "stdio.h"
#include "time.h"

//For time file
#include "ctime"

//For position.h, remember to add "using namespace std"
#include <vector>
using namespace std;

//For sixCameraStitching
using namespace cv;
using namespace cv::detail;

//For Camshift
#include "opencv2/video/tracking.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
#include <iostream>
#include <ctype.h>
//using namespace cv;
//using namespace std;