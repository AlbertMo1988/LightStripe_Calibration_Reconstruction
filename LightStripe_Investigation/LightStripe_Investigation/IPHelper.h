#pragma once
#include <string>
#include <vector>
#include "opencv2/opencv.hpp"
#pragma once

using namespace std;

class IPHelper
{
public:
	IPHelper();
	~IPHelper();

	void CVT_RGB2Gray(cv::Mat& src, cv::Mat& dst);
	void CVT_Gray2RGB(cv::Mat& src, cv::Mat& dst);

	void Seg_OTSU(cv::Mat& src, cv::Mat& dst, int OTSU_threshold_percentage=100);

	void Edge_Detection(cv::Mat& src, cv::Mat& dst, int model, int par1=0, int par2=0, int par3=0);

	void Hough_Line(cv::Mat& src, vector<cv::Point>& startPoint, vector<cv::Point>& endPoint);
	void Hough_LineP(cv::Mat& src, vector<cv::Point>& startPoint, vector<cv::Point>& endPoint, 
		double rho = 1, double theta = CV_PI / 180, int threshold = 80, double minLineLength = 0, double maxLineGap = 0);

	void Skeleton(const cv::Mat &src, cv::Mat &dst, const int iterations);

	void Extract_Points_Best_Upper(cv::Mat &src, vector<cv::Point> &Points, int threshold_value = 0);

private:

};
