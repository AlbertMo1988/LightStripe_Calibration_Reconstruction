#pragma once
#include <string>
#include <vector>
#include "opencv2/opencv.hpp"
#pragma once

using namespace std;
//#define DEBUG

class IPHelper_Calibration
{
public:
	IPHelper_Calibration();
	IPHelper_Calibration(cv::Size input_image_Boardsize, double input_image_SquareSize, int input_calibration_pattern_type);
	~IPHelper_Calibration();
	
	void UnDistortImage(cv::Mat& inputImage, cv::Mat& outputImage, cv::Mat& camera_matrix, cv::Mat& camera_distortion);

	void Load_Single_Camera_Image_Path(vector<string>& out_image_path, string file_image_path);
	void Find_Single_Camera_Image_Corners(vector<vector<cv::Point2f>>& out_image_corners_points);
	void Find_Single_Camera_Image_Corners_Standalone(cv::Size input_image_Boardsize, int input_calibration_pattern_type, cv::Mat inputImage, vector<vector<cv::Point2f>>& out_image_corners_points);
	bool RunCalibration(cv::Mat& cameraMatrix, cv::Mat& distCoeffs, vector<cv::Mat>& rvecs, vector<cv::Mat>& tvecs,
		vector<float>& reprojErrs, double& totalAvgErr);
	void SaveCameraParams(string file_path, bool save_every_image_detail=false);
	void LoadCameraParams(string xmlCalibrationFilePath, cv::Mat& cameraMatrix, cv::Mat& distCoeffs);
	void LoadCameraSize(string xmlCalibrationFilePath, cv::Size& output_BoardSize, double& output_SquareSize);
protected:
private:
	vector<string> class_private_image_file_paths;
	cv::Size class_private_image_boardSize;
	double class_private_squareSize;
	cv::Mat class_private_cameraMatrix, class_private_distCoeffs;
	cv::Size class_private_imageSize;
	int class_private_calibration_type;
	vector<vector<cv::Point2f> > class_private_imagePoints;
	vector<cv::Mat> class_private_rvecs, class_private_tvecs;
	vector<float> class_private_reprojErrs;
	double class_private_totalAvgErr;
};