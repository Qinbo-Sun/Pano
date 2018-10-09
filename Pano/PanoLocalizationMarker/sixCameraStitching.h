#pragma once

#define CAMERA1C -40
#define CAMERA1R 20
#define CAMERA2C -40
#define CAMERA2R 0
#define CAMERA3C -40
#define CAMERA3R -15
#define CAMERA4C 0
#define CAMERA4R 0
#define CAMERA5C 0
#define CAMERA5R 0
#define CAMERA6C 0
#define CAMERA6R 0

#define CAMERA1X 37
#define CAMERA1Y -30
#define CAMERA2X 40
#define CAMERA2Y 0
#define CAMERA3X 35
#define CAMERA3Y 20
#define CAMERA4X -23//2//BIG-right
#define CAMERA4Y -23//-34 //BIG-UP
#define CAMERA5X 0
#define CAMERA5Y 0
#define CAMERA6X 7
#define CAMERA6Y 0//4

//#define CAMERA1C -10
//#define CAMERA1R 15
//#define CAMERA2C -40
//#define CAMERA2R 0
//#define CAMERA3C -40
//#define CAMERA3R -15
//#define CAMERA4C 0
//#define CAMERA4R 0
//#define CAMERA5C 0
//#define CAMERA5R 0
//#define CAMERA6C 0
//#define CAMERA6R 0
//
//#define CAMERA1X 0//37
//#define CAMERA1Y 0//-30
//#define CAMERA2X 0//40
//#define CAMERA2Y 0
//#define CAMERA3X 0//35
//#define CAMERA3Y 0//20
//#define CAMERA4X -23//2//BIG-right
//#define CAMERA4Y -23//-34 //BIG-UP
//#define CAMERA5X 0
//#define CAMERA5Y 0
//#define CAMERA6X 7
//#define CAMERA6Y 0//4

Mat readhomography(const char* homo_xml) {
	Mat homo;
	FileStorage fs(homo_xml, cv::FileStorage::READ);
	if (!fs.isOpened()) {
		std::cout << "Save File Failed!" << std::endl;
		std::exit(1);
	}
	else {
		fs["homography"] >> homo;
		return homo;
	}
}

Mat sixStitching(VideoCapture capture_01, VideoCapture capture_02, VideoCapture capture_03, VideoCapture capture_04, VideoCapture capture_05, VideoCapture capture_06, Mat homo4to5, Mat homo6to5, Mat homo2to5, Mat homo3to2, Mat homo1to2) {
	//Defination
	Mat camera1, camera2, camera3, camera4, camera5, camera6;
	Mat result_s, result_mask;
	Mat allPanoImg;
	capture_01 >> camera1;
	capture_02 >> camera2;
	capture_03 >> camera3;
	capture_04 >> camera4;
	capture_05 >> camera5;
	capture_06 >> camera6;

	//New shift matrix
	Mat shf4to5Mat =
		(Mat_<double>(3, 3) << 1.0, 0, CAMERA4X, 0, 1.0, camera5.rows + CAMERA4Y, 0, 0, 1.0);
	Mat shf6to5Mat =
		(Mat_<double>(3, 3) << 1.0, 0, CAMERA6X, 0, 1.0, -1 * camera5.rows + CAMERA6Y, 0, 0, 1.0);
	Mat shf2to5Mat =
		(Mat_<double>(3, 3) << 1.0, 0, -1 * camera5.cols + CAMERA2X, 0, 1.0, -1 * camera5.rows + CAMERA2Y, 0, 0, 1.0);
	Mat shf3to2Mat =
		(Mat_<double>(3, 3) << 1.0, 0, -1 * camera5.cols + CAMERA3X, 0, 1.0, -2 * camera5.rows + CAMERA3Y, 0, 0, 1.0);
	Mat shf1to2Mat =
		(Mat_<double>(3, 3) << 1.0, 0, -1 * camera5.cols + CAMERA1X, 0, 1.0, CAMERA1Y, 0, 0, 1.0);

	//New Mask for each view
	Mat test5Mask(camera5.size(), CV_8U);
	test5Mask(Rect(0, 0, test5Mask.cols, test5Mask.rows)).setTo(255);
	Mat test4Mask(camera4.size(), CV_8U);
	test4Mask(Rect(0, 0, test4Mask.cols, test4Mask.rows)).setTo(255);
	Mat test6Mask(camera6.size(), CV_8U);
	test6Mask(Rect(0, 0, test6Mask.cols, test6Mask.rows)).setTo(255);
	Mat test2Mask(camera2.size(), CV_8U);
	test2Mask(Rect(0, 0, test2Mask.cols, test2Mask.rows)).setTo(255);
	Mat test3Mask(camera3.size(), CV_8U);
	test3Mask(Rect(0, 0, test3Mask.cols, test3Mask.rows)).setTo(255);
	Mat test1Mask(camera1.size(), CV_8U);
	test1Mask(Rect(0, 0, test1Mask.cols, test1Mask.rows)).setTo(255);

	//WarpPerspective act on Mask and Camera
	warpPerspective(test4Mask, test4Mask, shf4to5Mat*homo4to5,
		Size(camera4.cols, camera4.rows));
	warpPerspective(camera4, camera4, shf4to5Mat*homo4to5,
		Size(camera4.cols, camera4.rows));
	warpPerspective(test6Mask, test6Mask, shf6to5Mat*homo6to5,
		Size(camera6.cols, camera6.rows));
	warpPerspective(camera6, camera6, shf6to5Mat*homo6to5,
		Size(camera6.cols, camera6.rows));
	warpPerspective(test2Mask, test2Mask, shf2to5Mat*homo2to5,
		Size(camera2.cols, camera2.rows));
	warpPerspective(camera2, camera2, shf2to5Mat*homo2to5,
		Size(camera2.cols, camera2.rows));
	warpPerspective(test3Mask, test3Mask, shf3to2Mat*homo3to2,
		Size(camera3.cols, camera3.rows));
	warpPerspective(camera3, camera3, shf3to2Mat*homo3to2,
		Size(camera3.cols, camera3.rows));
	warpPerspective(test1Mask, test1Mask, shf1to2Mat*homo1to2,
		Size(camera1.cols, camera1.rows));
	warpPerspective(camera1, camera1, shf1to2Mat*homo1to2,
		Size(camera1.cols, camera1.rows));
	
	//Blender here
	MultiBandBlender blender(true, 4);
	blender.prepare(Rect(0, 0, camera5.cols * 2, camera5.rows * 3));
	blender.feed(camera5, test5Mask, Point(0 + CAMERA5C, camera5.rows + CAMERA5R));
	blender.feed(camera6, test6Mask, Point(0 +CAMERA6C, camera5.rows * 2 + CAMERA6R));
	blender.feed(camera4, test4Mask, Point(0 + CAMERA4C, 0 + CAMERA4R));
	blender.feed(camera1, test1Mask, Point(camera5.cols + CAMERA1C, 0 + CAMERA1R));
	blender.feed(camera2, test2Mask, Point(camera5.cols + CAMERA2C, camera5.rows + CAMERA2R));
	blender.feed(camera3, test3Mask, Point(camera5.cols + CAMERA3C, camera5.rows * 2 + CAMERA3R));
	blender.blend(result_s, result_mask);

	//End
	result_s.convertTo(allPanoImg, CV_8U);
	return allPanoImg;
}


