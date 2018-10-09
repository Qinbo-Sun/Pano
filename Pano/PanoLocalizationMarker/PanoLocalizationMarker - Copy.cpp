// PanoLocalizationMarker.cpp : Defines the entry point for the console application.
//
// PanoLocalization.cpp: 定义控制台应用程序的入口点。
//
// PanoImage.cpp: 定义控制台应用程序的入口点。
//
#include "head.h"
#include "stardb.h"
#include "sixCameraStitching.h"
#include "position.h"

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

const char* keys =
{
	"{1|  | 0 | camera number}"
};

string int2string(int value)
{
	stringstream ss;
	ss << value;
	return ss.str();
}

int main(int argc, char const* argv[]) {
	//Check command line
	if (argc != 6) {
		printf(
			"Usage: ./stitching 4to5.xml 6to5.xml LeftToCamera2.xml "
			"LeftToCamera3.xml Camera1ToLeft.xml output.jpg\n");
		return 1;
	}

	//Create time-named file 
	time_t t = std::time(0);
	struct tm * nowf = std::localtime(&t);
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
	file_nameVm= "data\\" + file_name + "_Marker.avi";
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
	//Define VideoCapture
	VideoCapture capture_01(6);//4//2
	VideoCapture capture_02(5);//3//5
	VideoCapture capture_03(0);//0//2
	VideoCapture capture_04(2);//5//4
	VideoCapture capture_05(4);//2//6
	VideoCapture capture_06(3);//1//3

	//Define homography matrix, and the PanoImg
	Mat homo4to5, homo6to5, homo2to5, homo3to2, homo1to2;
	Mat allPanoImg;

	//Read homography matrix
	homo4to5 = readhomography(argv[1]);
	homo6to5 = readhomography(argv[2]);
	homo2to5 = readhomography(argv[3]);
	homo3to2 = readhomography(argv[4]);
	homo1to2 = readhomography(argv[5]);


	//Create video file
	CvVideoWriter *video = NULL;
	CvVideoWriter *videoM = NULL;
	CvVideoWriter *videoMs = NULL;
	video = cvCreateVideoWriter(c_s, CV_FOURCC('X', 'V', 'I', 'D'), 9, cvSize(1280, 1440));
	videoM = cvCreateVideoWriter(c_m, CV_FOURCC('X', 'V', 'I', 'D'), 9, cvSize(1280, 1440));
	videoMs = cvCreateVideoWriter(c_ms, CV_FOURCC('X', 'V', 'I', 'D'), 9, cvSize(100, 100));
	if (video)
	{
		cout << "The video has been created" << endl;
	}

	//Tuple tuple;
	Tuple tuple;
	Tuple tupleG;
	int tempYPx = 0;
	int tempYPy = 0;
	int tempRPx = 0;
	int tempRPy = 0;

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

	// ------------------stitching each frame--------------------------------
	while (true) {
		// measure performance
		clock_t t;
		t = clock();
		allPanoImg = sixStitching(capture_01, capture_02, capture_03, capture_04, capture_05, capture_06, homo4to5, homo6to5, homo2to5, homo3to2, homo1to2);
		namedWindow("all", CV_WINDOW_NORMAL);
		imshow("all", allPanoImg);
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
				printf("Time:%f, PositionX:%f, PositionY:%f\n",passedTime, trackBox.center.x, trackBox.center.y);
				temp_x = trackBox.center.x;
				temp_y = trackBox.center.y;
				ModifyData(passedTime, temp_x, temp_y);

				dataRecorder
					<< " tConsume " << passedTime
					<< " X " << trackBox.center.x
					<< " Y " << trackBox.center.y
					<< "\n";

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
		//cvReleaseImage(&pall);
	}
	FreeConnect();
	//Close position data file
	dataRecorder.close();
	return 0;
}
