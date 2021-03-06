// PreCalibrate.cpp : This file contains the 'main' function. Program execution begins and ends there.
//

// Camera.cpp: 定义控制台应用程序的入口点。
//
#pragma warning(disable:4996)
#include <opencv2\core\core.hpp>
#include <opencv\highgui.h>
#include <opencv2\opencv.hpp>

#include "ctime"
#include "time.h"
#define _CRT_SECURE_NO_WARNINGS 1

#define CAMERA4C 30
#define CAMERA4R 30
#define CAMERA5C 0-12//4
#define CAMERA5R 0-150//4
#define CAMERA6C 0+24//5
#define CAMERA6R 0-162//5
#define CAMERA1C 0-243//4
#define CAMERA1R 0+9//4
#define CAMERA2C 0-240//5
#define CAMERA2R 0+13//5
#define CAMERA3C 0-290//6//252
#define CAMERA3R 0-13//6


using namespace cv;
using namespace std;

IplImage fra;
IplImage fra1;
IplImage fra2;
IplImage fra3;
IplImage fra4;
IplImage fra5;
IplImage fra6;
IplImage *p3 = NULL;
IplImage *p71 = NULL;
IplImage *p72 = NULL;
IplImage *p73 = NULL;
IplImage *p74 = NULL;
IplImage *p75 = NULL;
IplImage *p76 = NULL;

string int2string(int value)
{
	stringstream ss;
	ss << value;
	return ss.str();
}

int main()
{

	//Read the camera.
	VideoCapture capture_01(6);//4
	VideoCapture capture_02(5);//5
	VideoCapture capture_03(0);//0
	VideoCapture capture_04(2);
	VideoCapture capture_05(4);//1//4
	VideoCapture capture_06(3);//2//3
	capture_01.set(CV_CAP_PROP_FRAME_WIDTH, 640);
	capture_01.set(CV_CAP_PROP_FRAME_HEIGHT, 480);
	capture_02.set(CV_CAP_PROP_FRAME_WIDTH, 640);
	capture_02.set(CV_CAP_PROP_FRAME_HEIGHT, 480);
	capture_03.set(CV_CAP_PROP_FRAME_WIDTH, 640);
	capture_03.set(CV_CAP_PROP_FRAME_HEIGHT, 480);
	capture_04.set(CV_CAP_PROP_FRAME_WIDTH, 640);
	capture_04.set(CV_CAP_PROP_FRAME_HEIGHT, 480);
	capture_05.set(CV_CAP_PROP_FRAME_WIDTH, 640);
	capture_05.set(CV_CAP_PROP_FRAME_HEIGHT, 480);
	capture_06.set(CV_CAP_PROP_FRAME_WIDTH, 640);
	capture_06.set(CV_CAP_PROP_FRAME_HEIGHT, 480);
	//VideoCapture capture_07(6);//3//1
	namedWindow("All");
	namedWindow("v1");
	namedWindow("v2");
	namedWindow("v3");
	namedWindow("v4");//CV_WINDOW_NORMAL
	namedWindow("v5");
	namedWindow("v6");
	//namedWindow("v7");
	time_t t = std::time(0);
	struct tm * nowf = std::localtime(&t);
	string file_nameV;
	string file_nameAll;
	string file_nameV1;
	string file_nameV2;
	string file_nameV3;
	string file_nameV4;
	string file_nameV5;
	string file_nameV6;
	file_nameV = "video\\" + int2string(nowf->tm_year + 1900) +
		'-' + int2string(nowf->tm_mon + 1) +
		'-' + int2string(nowf->tm_mday) +
		'-' + int2string(nowf->tm_hour) +
		'-' + int2string(nowf->tm_min) +
		'-' + int2string(nowf->tm_sec);
	file_nameAll = file_nameV + "_all" + ".avi";
	file_nameV1 = file_nameV + "_01" + ".avi";
	file_nameV2 = file_nameV + "_02" + ".avi";
	file_nameV3 = file_nameV + "_03" + ".avi";
	file_nameV4 = file_nameV + "_04" + ".avi";
	file_nameV5 = file_nameV + "_05" + ".avi";
	file_nameV6 = file_nameV + "_06" + ".avi";

	const char *c_s = file_nameAll.c_str();
	const char *c_s1 = file_nameV1.c_str();
	const char *c_s2 = file_nameV2.c_str();
	const char *c_s3 = file_nameV3.c_str();
	const char *c_s4 = file_nameV4.c_str();
	const char *c_s5 = file_nameV5.c_str();
	const char *c_s6 = file_nameV6.c_str();

	const char *c_ss = file_nameV.data();
	Mat temp;
	capture_04 >> temp;
	CvVideoWriter *video = NULL;
	CvVideoWriter *video1 = NULL;
	CvVideoWriter *video2 = NULL;
	CvVideoWriter *video3 = NULL;
	CvVideoWriter *video4 = NULL;
	CvVideoWriter *video5 = NULL;
	CvVideoWriter *video6 = NULL;
	video = cvCreateVideoWriter(c_s, CV_FOURCC('X', 'V', 'I', 'D'), 9, cvSize(temp.cols + temp.cols + 1 + CAMERA4C, temp.rows * 3 + CAMERA4R));
	video1 = cvCreateVideoWriter(c_s1, CV_FOURCC('X', 'V', 'I', 'D'), 9, cvSize(temp.cols, temp.rows));
	video2 = cvCreateVideoWriter(c_s2, CV_FOURCC('X', 'V', 'I', 'D'), 9, cvSize(temp.cols, temp.rows));
	video3 = cvCreateVideoWriter(c_s3, CV_FOURCC('X', 'V', 'I', 'D'), 9, cvSize(temp.cols, temp.rows));
	video4 = cvCreateVideoWriter(c_s4, CV_FOURCC('X', 'V', 'I', 'D'), 9, cvSize(temp.cols, temp.rows));
	video5 = cvCreateVideoWriter(c_s5, CV_FOURCC('X', 'V', 'I', 'D'), 9, cvSize(temp.cols, temp.rows));
	video6 = cvCreateVideoWriter(c_s6, CV_FOURCC('X', 'V', 'I', 'D'), 9, cvSize(temp.cols, temp.rows));
	if (video)
	{
		cout << "The video has been created" << endl;
	}
	Mat frameAll(temp.rows * 3 + CAMERA4R, temp.cols + temp.cols + 1 + CAMERA4C, temp.type());
	fra = IplImage(frameAll);
	p3 = &(fra);
	while (true)
	{
		time_t t = std::time(0);
		struct tm * nowf = std::localtime(&t);
		string file_name;
		string file_name01;
		string file_name02;
		string file_name03;
		string file_name04;
		string file_name05;
		string file_name06;
		file_name = "pic\\" + int2string(nowf->tm_year + 1900) +
			'-' + int2string(nowf->tm_mon + 1) +
			'-' + int2string(nowf->tm_mday) +
			'-' + int2string(nowf->tm_hour) +
			'-' + int2string(nowf->tm_min) +
			'-' + int2string(nowf->tm_sec);
		file_name01 = file_name + "_01" + ".jpg";
		file_name02 = file_name + "_02" + ".jpg";
		file_name03 = file_name + "_03" + ".jpg";
		file_name04 = file_name + "_04" + ".jpg";
		file_name05 = file_name + "_05" + ".jpg";
		file_name06 = file_name + "_06" + ".jpg";


		Mat frame_01, frame_02, frame_03, frame_04, frame_05, frame_06, frame_07;
		Mat frame_11, frame_12, frame_13, frame_14, frame_15, frame_16;
		capture_01 >> frame_01;
		capture_02 >> frame_02;
		capture_03 >> frame_03;
		capture_04 >> frame_04;
		capture_05 >> frame_05;
		capture_06 >> frame_06;

		fra1 = IplImage(frame_01);
		fra2 = IplImage(frame_02);
		fra3 = IplImage(frame_03);
		fra4 = IplImage(frame_04);
		fra5 = IplImage(frame_05);
		fra6 = IplImage(frame_06);
		p71 = &(fra1);
		p72 = &(fra2);
		p73 = &(fra3);
		p74 = &(fra4);
		p75 = &(fra5);
		p76 = &(fra6);
		//capture_07 >> frame_07;
		//translateTransform2(frame_01, frame_11, 0, 50);


		/*Mat allFrame = frame_11;
		Mat frameAll(frame_03.rows*3, frame_03.cols*2+200, frame_03.type());
		Mat imageROI5;
		Mat imageROI2;
		Mat imageROI3;
		Mat imageROI1;
		Mat imageROI6;
		Mat imageROI4;
		imageROI5 = frameAll(Rect(frame_03.cols+15, 177, frame_05.cols, frame_05.rows));
		imageROI2 = frameAll(Rect(frame_03.cols, frame_03.rows, frame_02.cols, frame_02.rows));
		imageROI3 = frameAll(Rect(frame_03.cols+99, frame_03.rows*2-184, frame_03.cols, frame_03.rows));
		imageROI1 = frameAll(Rect(0, 0, frame_03.cols, frame_03.rows));
		imageROI6 = frameAll(Rect(0, frame_03.rows, frame_06.cols, frame_06.rows));
		imageROI4 = frameAll(Rect(0, frame_03.rows*2, frame_04.cols, frame_04.rows));
		frame_05.copyTo(imageROI5);
		frame_02.copyTo(imageROI2);
		frame_03.copyTo(imageROI3);
		frame_06.copyTo(imageROI1);
		frame_06.copyTo(imageROI6);
		frame_04.copyTo(imageROI4);*/

		//addWeighted(imageROI5, 1, frame_05, 0.3, 0, imageROI5);
		//frame_05.colRange(0,frame_05.cols).copyTo(frameAll.colRange(frame_05.rows, frame_05.rows*2));
		//frame_05.rowRange(0, frame_05.rows).copyTo(frameAll.rowRange(0, frame_05.rows));
		//frame_02.rowRange(0, frame_02.rows).copyTo(frameAll.rowRange(frame_03.rows, frame_03.rows*2));
		//frame_03.rowRange(0, frame_03.rows).copyTo(frameAll.rowRange(frame_03.rows * 2, frameAll.rows));
		//imshow("All", frameAll);



		Mat allFrame = frame_11;


		Mat imageROI1;
		Mat imageROI2;
		Mat imageROI3;
		Mat imageROI4;
		Mat imageROI5;
		Mat imageROI6;
		imageROI4 = frameAll(Rect(0 + CAMERA4C, 0 + CAMERA4R, frame_04.cols, frame_04.rows));
		imageROI5 = frameAll(Rect(0 + CAMERA5C + CAMERA4C, frame_04.rows + CAMERA5R + CAMERA4R, frame_04.cols, frame_04.rows));
		imageROI6 = frameAll(Rect(0 + CAMERA6C + CAMERA4C + CAMERA5C, frame_04.rows * 2 + CAMERA6R + CAMERA5R + CAMERA4R, frame_04.cols, frame_04.rows));
		imageROI1 = frameAll(Rect(frame_04.cols + CAMERA1C + CAMERA4C, 0 + CAMERA1R + CAMERA4R, frame_04.cols, frame_04.rows));
		imageROI2 = frameAll(Rect(frame_04.cols + CAMERA2C + CAMERA4C + CAMERA5C, frame_04.rows + CAMERA2R + CAMERA4R + CAMERA5R, frame_04.cols, frame_04.rows));
		imageROI3 = frameAll(Rect(frame_04.cols + CAMERA3C + CAMERA4C + CAMERA5C + CAMERA6C, frame_04.rows * 2 + CAMERA3R + CAMERA4R + CAMERA5R + CAMERA6R, frame_04.cols, frame_04.rows));
		cvWriteFrame(video, p3);
		cvWriteFrame(video1, p71);
		cvWriteFrame(video2, p72);
		cvWriteFrame(video3, p73);
		cvWriteFrame(video4, p74);

		cvWriteFrame(video5, p75);
		cvWriteFrame(video6, p76);



		frame_03.copyTo(imageROI3);
		frame_06.copyTo(imageROI6);
		frame_02.copyTo(imageROI2);
		frame_01.copyTo(imageROI1);
		frame_05.copyTo(imageROI5);
		frame_04.copyTo(imageROI4);

		//frame_05.colRange(1, frame_05.cols-1).copyTo(frameAll.colRange(1, frame_05.cols-1));
		//frame_05.rowRange(0, frame_05.rows).copyTo(frameAll.rowRange(0, frame_04.rows));
		//addWeighted(imageROI5, 1, frame_05, 0.3, 0, imageROI5);
		//frame_05.colRange(0, frame_05.cols).copyTo(frameAll.colRange(0, frame_04.cols));
		//frame_05.rowRange(0, frame_05.rows).copyTo(frameAll.rowRange(0, frame_04.rows));
		//frame_02.rowRange(0, frame_02.rows).copyTo(frameAll.rowRange(frame_03.rows, frame_03.rows*2));
		//frame_03.rowRange(0, frame_03.rows).copyTo(frameAll.rowRange(frame_03.rows * 2, frameAll.rows));
		//Mat Stitching(frame_04.rows, frame_04.cols*2, frame_04.type());
		//frame_05.colRange(0, frame_05.cols).copyTo(Stitching.colRange(0, frame_05.cols));
		//frame_05.rowRange(0, frame_05.rows-20).copyTo(Stitching.rowRange(0, frame_05.rows - 20));
		//frame_06.colRange(0, frame_06.cols).copyTo(Stitching.colRange(frame_05.cols, Stitching.cols));
		//imshow("result", Stitching);
		imshow("All", frameAll);


		imshow("v1", frame_01);
		imshow("v2", frame_02);
		imshow("v3", frame_03);
		imshow("v4", frame_04);
		imshow("v5", frame_05);
		imshow("v6", frame_06);


		//imshow("v7", frame_07);
		imwrite(file_name01, frame_01);
		imwrite(file_name02, frame_02);
		imwrite(file_name03, frame_03);
		imwrite(file_name04, frame_04);
		imwrite(file_name05, frame_05);
		imwrite(file_name06, frame_06);
		waitKey(30);
	}
	return 0;
}



// Run program: Ctrl + F5 or Debug > Start Without Debugging menu
// Debug program: F5 or Debug > Start Debugging menu

// Tips for Getting Started: 
//   1. Use the Solution Explorer window to add/manage files
//   2. Use the Team Explorer window to connect to source control
//   3. Use the Output window to see build output and other messages
//   4. Use the Error List window to view errors
//   5. Go to Project > Add New Item to create new code files, or Project > Add Existing Item to add existing code files to the project
//   6. In the future, to open this project again, go to File > Open > Project and select the .sln file
