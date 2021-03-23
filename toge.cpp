
#include<iostream>
#include<opencv2/opencv.hpp>
#include<opencv2/highgui/highgui.hpp>
#include<opencv2/imgproc/imgproc.hpp>
#include<opencv2/core/core.hpp>
#include<cmath>
//vector<vector<Point>>contours;
//vector<Vec4i>hierarchy;
using namespace cv;
using namespace std;

const int max_value_H = 360 / 2;
const int max_value = 255;
int low_H = 0, low_L = 0, low_S = 0;
int high_H = max_value_H, high_L = max_value, high_S = max_value;
//ÑÕÉ«Çø¼ä
String HLSname = "HLS";
Mat clo_HLS, clo_threshold;

static void on_low_H_thresh_trackbar(int, void*)
{
	low_H = min(high_H - 1, low_H);
	setTrackbarPos("Low H", HLSname, low_H);
}
static void on_high_H_thresh_trackbar(int, void*)
{
	high_H = max(high_H, low_H + 1);
	setTrackbarPos("High H", HLSname, high_H);
}
static void on_low_L_thresh_trackbar(int, void*)
{
	low_L = min(high_L - 1, low_L);
	setTrackbarPos("Low L", HLSname, low_L);
}
static void on_high_L_thresh_trackbar(int, void*)
{
	high_L = max(high_L, low_L + 1);
	setTrackbarPos("High L", HLSname, high_L);
}
static void on_low_S_thresh_trackbar(int, void*)
{
	low_S = min(high_S - 1, low_S);
	setTrackbarPos("Low V", HLSname, low_S);
}
static void on_high_S_thresh_trackbar(int, void*)
{
	high_S = max(high_S, low_S + 1);
	setTrackbarPos("High S", HLSname, high_S);
}

int thresholdvalue = 100;
int thresholdtype = 3;

Mat dst;
void threshold( int, void*)
{
	Mat gray;

		threshold(gray, dst, thresholdvalue, 255, thresholdtype);
	
}

int main()
{  
	VideoCapture capture(1);
	Mat src, gray;
	while (1)
	{
		capture >> src;
		cvtColor(src, gray, COLOR_BGR2GRAY);
		namedWindow("WINDOW_NAME", WINDOW_AUTOSIZE);
		createTrackbar("modle", "WINDOW_NAME", &thresholdtype, 4, threshold);
		createTrackbar("number", "WINDOW_NAME", &thresholdvalue, 255, threshold);
		threshold(gray, dst, thresholdvalue, 255, thresholdtype);
		imshow("capture", src);
		imshow("WINDOW_NAME", dst);
			char(key) = (char)waitKey(1);
		if (key == 27)
			break;
	}
	return 0;
}
