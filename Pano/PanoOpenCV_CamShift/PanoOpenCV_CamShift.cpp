// PanoOpenCV_CamShift.cpp : This file contains the 'main' function. Program execution begins and ends there.
//

// PanoOpenCV.cpp : This file contains the 'main' function. Program execution begins and ends there.
//

// Pano_Larryu.cpp : This file contains the 'main' function. Program execution begins and ends there.
//

// panoTest.cpp: 定义控制台应用程序的入口点。
//

//#include"stdafx.h"
//#include<opencv2\opencv.hpp>
//using namespace cv;
//int main()
//{
//	Mat picture = imread("pic//2018-7-18-15-55-19_01.jpg");
//	imshow("test demo", picture);
//	waitKey(20150901);
//}

#pragma warning( disable : 4996) 

#include <iostream>
#include <fstream>
#include <string>
#include "opencv2/opencv_modules.hpp"
#include <opencv2/core/utility.hpp>
#include "opencv2/imgcodecs.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/stitching/detail/autocalib.hpp"
#include "opencv2/stitching/detail/blenders.hpp"
#include "opencv2/stitching/detail/timelapsers.hpp"
#include "opencv2/stitching/detail/camera.hpp"
#include "opencv2/stitching/detail/exposure_compensate.hpp"
#include "opencv2/stitching/detail/matchers.hpp"
#include "opencv2/stitching/detail/motion_estimators.hpp"
#include "opencv2/stitching/detail/seam_finders.hpp"
#include "opencv2/stitching/detail/warpers.hpp"
#include "opencv2/stitching/warpers.hpp"
#include "utils.h"

//For time file
#include "ctime"
//For Camshift
#include "opencv2/video/tracking.hpp"

#include <iostream>
#include <ctype.h>
//For Database
#include "stardb.h"


#define ENABLE_LOG 1
#define LOG(msg) std::cout << msg
#define LOGLN(msg) std::cout << msg << std::endl

//Record the position data
ofstream dataRecorder;
IplImage fraAll;
IplImage fraImage;
IplImage fraSail;
IplImage *pAll = NULL;
IplImage *pImage = NULL;
IplImage *pSail = NULL;
//Global var
Mat image;
bool backprojMode = false;
bool selectObject = false;
int trackObject = 0;
bool showHist = true;
Point origin;
Rect selection;
int vmin = 10, vmax = 256, smin = 30;

//onMouse CallBack
static void onMouse(int event, int x, int y, int, void*)
{
	if (selectObject)
	{
		selection.x = MIN(x, origin.x);
		selection.y = MIN(y, origin.y);
		selection.width = std::abs(x - origin.x);
		selection.height = std::abs(y - origin.y);

		selection &= Rect(0, 0, image.cols, image.rows);
	}
	switch (event)
	{
	case EVENT_LBUTTONDOWN:
		origin = Point(x, y);
		selection = Rect(x, y, 0, 0);
		selectObject = true;
		break;
	case EVENT_LBUTTONUP:
		selectObject = false;
		if (selection.width > 0 && selection.height > 0)
			trackObject = -1;
		break;
	}
}

string int2string(int value)
{
	stringstream ss;
	ss << value;
	return ss.str();
}

int main(int argc, char* argv[])
{
#if ENABLE_LOG
	//int64 app_start_time = getTickCount();
#endif

#if 0
	cv::setBreakOnError(true);
#endif

	int retval = parseCmdArgs(argc, argv);
	if (retval)
		return retval;

	// Check if have enough images
	int num_images = static_cast<int>(img_names.size());
	if (num_images < 2)
	{
		// LOGLN("Need more images");
		return -1;
	}

	double work_scale = 1, seam_scale = 1, compose_scale = 1;
	bool is_work_scale_set = false, is_seam_scale_set = false, is_compose_scale_set = false;

#if ENABLE_LOG
	int64 t = getTickCount();
#endif

	//Create time-named file 
	time_t tf = std::time(0);
	struct tm * nowf = std::localtime(&tf);
	string file_name;
	string file_nameV;
	string file_nameVm;
	string file_nameVms;
	//the name of bag file is better to be determined by the system time
	file_name = int2string(nowf->tm_year + 1900) +
		'-' + int2string(nowf->tm_mon + 1) +
		'-' + int2string(nowf->tm_mday) +
		'-' + int2string(nowf->tm_hour) +
		'-' + int2string(nowf->tm_min) +
		'-' + int2string(nowf->tm_sec);
	file_nameV = "data\\" + file_name + ".avi";
	file_nameVm = "data\\" + file_name + "_Marker.avi";
	file_nameVms = "data\\" + file_name + "_MarkerSail.avi";
	const char *c_s = file_nameV.c_str();
	const char *c_ss = file_nameV.data();
	const char *c_m = file_nameVm.c_str();
	const char *c_ms = file_nameVms.c_str();
	cout << file_name << endl;
	// Record data
	dataRecorder.open("data\\" + file_name + ".txt");

	//Database
	ConnectDatabase();

	//Initialize VideoCapture
	VideoCapture capture_01(6);//4//2
	VideoCapture capture_02(5);//3//5
	VideoCapture capture_03(0);//0//2
	VideoCapture capture_04(2);//5//4
	VideoCapture capture_05(4);//2//6
	VideoCapture capture_06(3);//1//3
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

	//Create video file
	CvVideoWriter *video = NULL;
	CvVideoWriter *videoM = NULL;
	CvVideoWriter *videoMs = NULL;
	video = cvCreateVideoWriter(c_s, CV_FOURCC('X', 'V', 'I', 'D'), 9, cvSize(1231, 1217));
	videoM = cvCreateVideoWriter(c_m, CV_FOURCC('X', 'V', 'I', 'D'), 9, cvSize(1231, 1217));
	videoMs = cvCreateVideoWriter(c_ms, CV_FOURCC('X', 'V', 'I', 'D'), 9, cvSize(100, 100));
	if (video)
	{
		cout << "The video has been created" << endl;
	}

	//time
	clock_t start;
	clock_t tempVel;
	float passedTime;

	//Rectbox center point
	float temp_x = 0;
	float temp_y = 0;

	//Camshift
	Rect trackWindow;
	int hsize = 16;
	float hranges[] = { 0,180 };
	const float* phranges = hranges;
	namedWindow("Histogram", CV_WINDOW_NORMAL);
	namedWindow("CamShift Demo", CV_WINDOW_NORMAL);
	setMouseCallback("CamShift Demo", onMouse, 0);
	createTrackbar("Vmin", "CamShift Demo", &vmin, 256, 0);
	createTrackbar("Vmax", "CamShift Demo", &vmax, 256, 0);
	createTrackbar("Smin", "CamShift Demo", &smin, 256, 0);
	Mat frame, hsv, hue, mask, hist, histimg = Mat::zeros(200, 320, CV_8UC3), backproj;
	bool paused = false;

	start = clock();

	while (true) {
		//star record
		int64 app_start_time = getTickCount();
		vector<Mat> image_temp(num_images);

		Mat full_img, img;
		vector<Mat> images(num_images);
		vector<Size> full_img_sizes(num_images);
		double seam_work_aspect = 1;

		for (int i = 0; i < num_images; ++i)
		{
			full_img = imread(img_names[i]);
			//full_img = image_temp[i];
			full_img_sizes[i] = full_img.size();

			if (full_img.empty())
			{
				// LOGLN("Can't open image " << img_names[i]);
				return -1;
			}
			if (work_megapix < 0)
			{
				img = full_img;
				work_scale = 1;
				is_work_scale_set = true;
			}
			else
			{
				if (!is_work_scale_set)
				{
					work_scale = min(1.0, sqrt(work_megapix * 1e6 / full_img.size().area()));
					is_work_scale_set = true;
				}
				resize(full_img, img, Size(), work_scale, work_scale);
				// cout << "First resize, scale: " << work_scale << endl;
			}
			if (!is_seam_scale_set)
			{
				seam_scale = min(1.0, sqrt(seam_megapix * 1e6 / full_img.size().area()));
				seam_work_aspect = seam_scale / work_scale;
				is_seam_scale_set = true;
			}

			resize(full_img, img, Size(), seam_scale, seam_scale);
			// cout << "Second resize, scale: " << seam_scale << endl;
			images[i] = img.clone();

			capture_01 >> images[0];
			capture_02 >> images[1];
			capture_03 >> images[2];
			capture_04 >> images[3];
			capture_05 >> images[4];
			capture_06 >> images[5];
		}

		// load indices
		vector<int> indices;
		FileStorage idx_fs("indices.xml", cv::FileStorage::READ);
		idx_fs["indices"] >> indices;
		// cout << "Loaded indices successfully." << endl;
		idx_fs.release();

		// Leave only images are from the same panorama
		vector<Mat> img_subset;
		vector<String> img_names_subset;
		vector<Size> full_img_sizes_subset;
		for (size_t i = 0; i < indices.size(); ++i)
		{
			img_names_subset.push_back(img_names[indices[i]]);
			img_subset.push_back(images[indices[i]]);
			full_img_sizes_subset.push_back(full_img_sizes[indices[i]]);
		}

		images = img_subset;
		img_names = img_names_subset;
		full_img_sizes = full_img_sizes_subset;

		// Check if we still have enough images
		num_images = static_cast<int>(img_names.size());
		if (num_images < 2)
		{
			// LOGLN("Need more images");
			return -1;
		}

		vector<CameraParams> cameras(num_images);
		float warped_image_scale;

		// load camera parameters
		string xml_file_name = "camerasParameters.xml";
		loadCameraParams(xml_file_name, cameras, warped_image_scale);
		// LOGLN("Loaded cameras parameters successfully.");
		// cout << "Test CameraParams: print focal, focal = " << cameras[0].focal << endl; 


		// LOGLN("Warping images (auxiliary)... ");
#if ENABLE_LOG
		t = getTickCount();
#endif

		vector<Point> corners(num_images);
		vector<UMat> masks_warped(num_images);
		vector<UMat> images_warped(num_images);
		vector<Size> sizes(num_images);
		vector<UMat> masks(num_images);

		// Preapre images masks
		for (int i = 0; i < num_images; ++i)
		{
			masks[i].create(images[i].size(), CV_8U);
			masks[i].setTo(Scalar::all(255));
		}

		// Warp images and their masks

		Ptr<WarperCreator> warper_creator;
#ifdef HAVE_OPENCV_CUDAWARPING
		if (try_cuda && cuda::getCudaEnabledDeviceCount() > 0)
		{
			if (warp_type == "plane")
				warper_creator = makePtr<cv::PlaneWarperGpu>();
			else if (warp_type == "cylindrical")
				warper_creator = makePtr<cv::CylindricalWarperGpu>();
			else if (warp_type == "spherical")
				warper_creator = makePtr<cv::SphericalWarperGpu>();
		}
		else
#endif
		{
			if (warp_type == "plane")
				warper_creator = makePtr<cv::PlaneWarper>();
			else if (warp_type == "affine")
				warper_creator = makePtr<cv::AffineWarper>();
			else if (warp_type == "cylindrical")
				warper_creator = makePtr<cv::CylindricalWarper>();
			else if (warp_type == "spherical")
				warper_creator = makePtr<cv::SphericalWarper>();
			else if (warp_type == "fisheye")
				warper_creator = makePtr<cv::FisheyeWarper>();
			else if (warp_type == "stereographic")
				warper_creator = makePtr<cv::StereographicWarper>();
			else if (warp_type == "compressedPlaneA2B1")
				warper_creator = makePtr<cv::CompressedRectilinearWarper>(2.0f, 1.0f);
			else if (warp_type == "compressedPlaneA1.5B1")
				warper_creator = makePtr<cv::CompressedRectilinearWarper>(1.5f, 1.0f);
			else if (warp_type == "compressedPlanePortraitA2B1")
				warper_creator = makePtr<cv::CompressedRectilinearPortraitWarper>(2.0f, 1.0f);
			else if (warp_type == "compressedPlanePortraitA1.5B1")
				warper_creator = makePtr<cv::CompressedRectilinearPortraitWarper>(1.5f, 1.0f);
			else if (warp_type == "paniniA2B1")
				warper_creator = makePtr<cv::PaniniWarper>(2.0f, 1.0f);
			else if (warp_type == "paniniA1.5B1")
				warper_creator = makePtr<cv::PaniniWarper>(1.5f, 1.0f);
			else if (warp_type == "paniniPortraitA2B1")
				warper_creator = makePtr<cv::PaniniPortraitWarper>(2.0f, 1.0f);
			else if (warp_type == "paniniPortraitA1.5B1")
				warper_creator = makePtr<cv::PaniniPortraitWarper>(1.5f, 1.0f);
			else if (warp_type == "mercator")
				warper_creator = makePtr<cv::MercatorWarper>();
			else if (warp_type == "transverseMercator")
				warper_creator = makePtr<cv::TransverseMercatorWarper>();
		}

		if (!warper_creator)
		{
			// cout << "Can't create the following warper '" << warp_type << "'\n";
			return 1;
		}

		// Ptr<RotationWarper> warper = warper_creator->create(static_cast<float>(warped_image_scale * seam_work_aspect));
		Ptr<RotationWarper> warper = warper_creator->create(static_cast<float>(warped_image_scale));
		// LOGLN("First warper scale: " << warped_image_scale);

		// cout << "success 0" << endl;

		//ADD TO:

		for (int i = 0; i < num_images; ++i)
		{
			Mat_<float> K;
			cameras[i].K().convertTo(K, CV_32F);
			float swa = (float)seam_work_aspect;
			K(0, 0) *= swa; K(0, 2) *= swa;
			K(1, 1) *= swa; K(1, 2) *= swa;

			corners[i] = warper->warp(images[i], K, cameras[i].R, INTER_LINEAR, BORDER_REFLECT, images_warped[i]);
			sizes[i] = images_warped[i].size();

			warper->warp(masks[i], K, cameras[i].R, INTER_NEAREST, BORDER_CONSTANT, masks_warped[i]);
		}

		// cout << "Success 1" << endl;

		vector<UMat> images_warped_f(num_images);
		for (int i = 0; i < num_images; ++i)
			images_warped[i].convertTo(images_warped_f[i], CV_32F);

		// LOGLN("Warping images, time: " << ((getTickCount() - t) / getTickFrequency()) << " sec");


		// LOGLN("Compositing...");
#if ENABLE_LOG
		t = getTickCount();
#endif

		Mat img_warped, img_warped_s;
		Mat dilated_mask, seam_mask, mask, mask_warped;
		Ptr<Blender> blender;
		Ptr<Timelapser> timelapser;
		//double compose_seam_aspect = 1;
		double compose_work_aspect = 1;

		/*----------------------v2 implementation, simplified version----------------------*/
		Size dst_sz = resultRoi(corners, sizes).size();
		float blend_width = sqrt(static_cast<float>(dst_sz.area())) * blend_strength / 100.f;

		blender = Blender::createDefault(blend_type, try_cuda);
		MultiBandBlender* mb = dynamic_cast<MultiBandBlender*>(blender.get());
		// mb->setNumBands(static_cast<int>(ceil(log(blend_width)/log(2.)) - 1.));
		mb->setNumBands(5);
		// LOGLN("Multi-band blender, number of bands: " << mb->numBands());

		blender->prepare(corners, sizes);
		/*----------------------v2 implementation, simplified version----------------------*/

		for (int img_idx = 0; img_idx < num_images; ++img_idx)
		{
			// LOGLN("Compositing image #" << indices[img_idx]+1);

			/*----------------------v2 implementation, simplified version----------------------*/
			blender->feed(images_warped[img_idx], masks_warped[img_idx], corners[img_idx]);
			// print cout image size
			// if (img_idx == 0) cout << "images_warped #" << img_idx << " size " << images_warped[img_idx].size() << endl;
			/*----------------------v2 implementation, simplified version----------------------*/


		}

		if (true)//!timelapse
		{
			
		}
		//Pano blender
		Mat result, result_mask;
		blender->blend(result, result_mask);
		// LOGLN("Compositing, time: " << ((getTickCount() - t) / getTickFrequency()) << " sec");
		Mat allPanoImg;
		imwrite(result_name, result);
		result.convertTo(allPanoImg, CV_8U);
		namedWindow("all", CV_WINDOW_NORMAL);
		imshow("all", allPanoImg);
		waitKey(30);
		// measure performance
		clock_t t;
		t = clock();
		fraAll = IplImage(allPanoImg);
		pAll = &(fraAll);
		tempVel = clock();
		passedTime = difftime(tempVel, start);
		passedTime = passedTime / 1000;
		time_t tPano = std::time(0);
		struct tm * nowf = std::localtime(&tPano);
		string timeDisplay;
		timeDisplay = int2string(nowf->tm_year + 1900) +
			'-' + int2string(nowf->tm_mon + 1) +
			'-' + int2string(nowf->tm_mday) +
			'-' + int2string(nowf->tm_hour) +
			'-' + int2string(nowf->tm_min) +
			'-' + int2string(nowf->tm_sec);
		const char *c_Display = timeDisplay.c_str();
		CvFont font;
		cvInitFont(&font, CV_FONT_HERSHEY_DUPLEX, 1, 1, 0, 2, 4);
		cvPutText(pAll, c_Display, cvPoint(100, 100), &font, cvScalar(255, 255, 255));
		cvWriteFrame(video, pAll);
		printf("Stitching takes %f seconds\n",
			float(clock() - t) / CLOCKS_PER_SEC);
		allPanoImg.copyTo(image);

		if (!paused)
		{
			cvtColor(image, hsv, COLOR_BGR2HSV);

			if (trackObject)
			{
				int _vmin = vmin, _vmax = vmax;

				inRange(hsv, Scalar(0, smin, MIN(_vmin, _vmax)),
					Scalar(180, 256, MAX(_vmin, _vmax)), mask);
				int ch[] = { 0, 0 };
				hue.create(hsv.size(), hsv.depth());
				mixChannels(&hsv, 1, &hue, 1, ch, 1);

				if (trackObject < 0)
				{
					Mat roi(hue, selection), maskroi(mask, selection);
					calcHist(&roi, 1, 0, maskroi, hist, 1, &hsize, &phranges);
					normalize(hist, hist, 0, 255, NORM_MINMAX);

					trackWindow = selection;
					trackObject = 1;

					histimg = Scalar::all(0);
					int binW = histimg.cols / hsize;
					Mat buf(1, hsize, CV_8UC3);
					for (int i = 0; i < hsize; i++)
						buf.at<Vec3b>(i) = Vec3b(saturate_cast<uchar>(i*180. / hsize), 255, 255);
					cvtColor(buf, buf, COLOR_HSV2BGR);

					for (int i = 0; i < hsize; i++)
					{
						int val = saturate_cast<int>(hist.at<float>(i)*histimg.rows / 255);
						rectangle(histimg, Point(i*binW, histimg.rows),
							Point((i + 1)*binW, histimg.rows - val),
							Scalar(buf.at<Vec3b>(i)), -1, 8);
					}
				}

				calcBackProject(&hue, 1, 0, hist, backproj, &phranges);
				backproj &= mask;
				RotatedRect trackBox = CamShift(backproj, trackWindow, TermCriteria(TermCriteria::EPS | TermCriteria::COUNT, 10, 1));
				if (trackWindow.area() <= 1)
				{
					int cols = backproj.cols, rows = backproj.rows, r = (MIN(cols, rows) + 5) / 6;
					trackWindow = Rect(trackWindow.x - r, trackWindow.y - r,
						trackWindow.x + r, trackWindow.y + r) &
						Rect(0, 0, cols, rows);
				}

				if (backprojMode)
					cvtColor(backproj, image, COLOR_GRAY2BGR);
				ellipse(image, trackBox, Scalar(0, 0, 255), 3, LINE_AA);
				printf("Time:%f, PositionX:%f, PositionY:%f\n", passedTime, trackBox.center.x, trackBox.center.y);
				temp_x = trackBox.center.x;
				temp_y = trackBox.center.y;
				ModifyData(passedTime, temp_x, temp_y);

				dataRecorder
					<< " tConsume " << passedTime
					<< " X " << trackBox.center.x
					<< " Y " << trackBox.center.y
					<< endl;

				//Create a region to detect the sail
				if (trackBox.center.x > 51 && trackBox.center.y > 51 && trackBox.center.x < 1280 - 51 && trackBox.center.y < 1440 - 51)
				{
					Mat sail(allPanoImg, Rect(trackBox.center.x - 50, trackBox.center.y - 50, 100, 100));
					fraSail = IplImage(sail);
					pSail = &(fraSail);
					cvWriteFrame(videoMs, pSail);
					namedWindow("sail", CV_WINDOW_NORMAL);
					imshow("sail", sail);
				}
			}
		}
		else if (trackObject < 0)
			paused = false;

		if (selectObject && selection.width > 0 && selection.height > 0)
		{
			Mat roi(image, selection);
			bitwise_not(roi, roi);
		}
		fraImage = IplImage(image);
		pImage = &(fraImage);
		cvWriteFrame(videoM, pImage);
		imshow("CamShift Demo", image);
		imshow("Histogram", histimg);
		char c = (char)waitKey(10);
		if (c == 27)
			break;
		switch (c)
		{
		case 'b':
			backprojMode = !backprojMode;
			break;
		case 'c':
			trackObject = 0;
			histimg = Scalar::all(0);
			break;
		case 'h':
			showHist = !showHist;
			if (!showHist)
				destroyWindow("Histogram");
			else
				namedWindow("Histogram", 1);
			break;
		case 'p':
			paused = !paused;
			break;
		default:
			;
		}
		LOGLN(image.cols << "  " << image.rows);
		LOGLN(allPanoImg.cols << "  " << allPanoImg.rows);

		// LOGLN("Finished, total time: " << ((getTickCount() - app_start_time) / getTickFrequency()) << " sec");
		double total_t = (getTickCount() - app_start_time) / getTickFrequency();
		double fps = 1.0 / total_t;
		LOGLN(fps << " fps");
	}
	FreeConnect();
	//Close position data file
	dataRecorder.close();
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

// Run program: Ctrl + F5 or Debug > Start Without Debugging menu
// Debug program: F5 or Debug > Start Debugging menu

// Tips for Getting Started: 
//   1. Use the Solution Explorer window to add/manage files
//   2. Use the Team Explorer window to connect to source control
//   3. Use the Output window to see build output and other messages
//   4. Use the Error List window to view errors
//   5. Go to Project > Add New Item to create new code files, or Project > Add Existing Item to add existing code files to the project
//   6. In the future, to open this project again, go to File > Open > Project and select the .sln file


// Run program: Ctrl + F5 or Debug > Start Without Debugging menu
// Debug program: F5 or Debug > Start Debugging menu

// Tips for Getting Started: 
//   1. Use the Solution Explorer window to add/manage files
//   2. Use the Team Explorer window to connect to source control
//   3. Use the Output window to see build output and other messages
//   4. Use the Error List window to view errors
//   5. Go to Project > Add New Item to create new code files, or Project > Add Existing Item to add existing code files to the project
//   6. In the future, to open this project again, go to File > Open > Project and select the .sln file
