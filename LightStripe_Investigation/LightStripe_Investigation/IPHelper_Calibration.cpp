#include "IPHelper_Calibration.h"
#include "FileHelper.h"
#include <io.h>
#include <stdlib.h>
#include <fstream>
#include <iostream>
#include <string>
#include <sstream>

using namespace std;

IPHelper_Calibration::IPHelper_Calibration(){}
IPHelper_Calibration::~IPHelper_Calibration(){}
IPHelper_Calibration::IPHelper_Calibration(cv::Size input_image_Boardsize, double input_image_SquareSize, int input_calibration_pattern_type)
{
	class_private_image_boardSize = input_image_Boardsize;
	class_private_squareSize = input_image_SquareSize;
	class_private_calibration_type = input_calibration_pattern_type;
}

void IPHelper_Calibration::UnDistortImage(cv::Mat& inputImage, cv::Mat& outputImage, cv::Mat& camera_matrix, cv::Mat& camera_distortion)
{
	cv::Mat map1, map2;
	cv::initUndistortRectifyMap(
		camera_matrix,
		camera_distortion,
		cv::Mat(),
		camera_matrix,
		inputImage.size(),
		CV_32FC1,
		map1, map2);
	cv::remap(inputImage, outputImage, map1, map2, cv::INTER_LINEAR);
}

void IPHelper_Calibration::Load_Single_Camera_Image_Path(vector<string>& out_image_path, string file_image_path)
{
	FileHelper fileHelper;
	fileHelper.getFileList(file_image_path, out_image_path,".bmp");
	class_private_image_file_paths = out_image_path;
}

void IPHelper_Calibration::Find_Single_Camera_Image_Corners(vector<vector<cv::Point2f>>& out_image_corners_points)
{
	//Pattern:
	//1: chessboard
	//2: circle
	//3: asymmetric circle pattern

	vector<vector<cv::Point2f> > imagePoints;
	cv::Mat cameraMatrix, distCoeffs;
	const cv::Scalar RED(0, 0, 255), GREEN(0, 255, 0);
	int imageNum = class_private_image_file_paths.size();
	cv::Size boardSize = class_private_image_boardSize;
	for (int i = 0; i < imageNum;i++)
	{
		cv::Mat view = cv::imread(class_private_image_file_paths[i], cv::IMREAD_GRAYSCALE);
		if (view.empty())
		{
			continue;
		}
		class_private_imageSize = view.size();  // Format input image.
		vector<cv::Point2f> pointBuf;
		bool found;
		int chessBoardFlags = cv::CALIB_CB_ADAPTIVE_THRESH + cv::CALIB_CB_NORMALIZE_IMAGE;
		switch (class_private_calibration_type) // Find feature points on the input format
		{
		case 1:
			found = cv::findChessboardCorners(view, boardSize, pointBuf, chessBoardFlags);
			break;
		case 2:
			found = cv::findCirclesGrid(view, boardSize, pointBuf);
			break;
		case 3:
			found = cv::findCirclesGrid(view, boardSize, pointBuf, cv::CALIB_CB_ASYMMETRIC_GRID);
			break;
		default:
			found = false;
			break;
		}
		if (found)
		{
			if (class_private_calibration_type==1)//Chessboard subpixel detection
			{
				cv::cornerSubPix(view, pointBuf, cv::Size(11, 11),
					cv::Size(-1, -1), cv::TermCriteria(cv::TermCriteria::EPS + cv::TermCriteria::COUNT, 30, 0.1));
			}
			imagePoints.push_back(pointBuf);
#ifdef _DEBUG
			//Draw
			cv::Mat colorView;
			cvtColor(view, colorView, cv::COLOR_GRAY2BGR);
			drawChessboardCorners(colorView, boardSize, cv::Mat(pointBuf), found);
			imshow("Successful", colorView);
			cvWaitKey();
#endif
		}
		else
		{
			string str = to_string(i);
			cv::imshow("Unsuccessful" + str, view);
			cvWaitKey();
		}
	}

	out_image_corners_points = imagePoints;
	class_private_imagePoints = imagePoints;
}

void IPHelper_Calibration::Find_Single_Camera_Image_Corners_Standalone(cv::Size input_image_Boardsize, int input_calibration_pattern_type, 
	cv::Mat inputImage, vector<vector<cv::Point2f>>& out_image_corners_points)
{
	//Pattern:
	//1: chessboard
	//2: circle
	//3: asymmetric circle pattern

	vector<vector<cv::Point2f> > imagePoints;
	const cv::Scalar RED(0, 0, 255), GREEN(0, 255, 0);

	vector<cv::Point2f> pointBuf;
	bool found;
	int chessBoardFlags = cv::CALIB_CB_ADAPTIVE_THRESH | cv::CALIB_CB_NORMALIZE_IMAGE;
	switch (input_calibration_pattern_type) // Find feature points on the input format
	{
	case 1:
		found = cv::findChessboardCorners(inputImage, input_image_Boardsize, pointBuf, chessBoardFlags);
		break;
	case 2:
		found = cv::findCirclesGrid(inputImage, input_image_Boardsize, pointBuf);
		break;
	case 3:
		found = cv::findCirclesGrid(inputImage, input_image_Boardsize, pointBuf, cv::CALIB_CB_ASYMMETRIC_GRID);
		break;
	default:
		found = false;
		break;
	}
	if (found)
	{
		if (class_private_calibration_type == 1)//Chessboard subpixel detection
		{
			cv::cornerSubPix(inputImage, pointBuf, cv::Size(11, 11),
				cv::Size(-1, -1), cv::TermCriteria(cv::TermCriteria::EPS + cv::TermCriteria::COUNT, 30, 0.1));
		}
		imagePoints.push_back(pointBuf);
#ifdef _DEBUG
		//Draw
		cv::Mat colorView;
		cvtColor(inputImage, colorView, cv::COLOR_GRAY2BGR);
		drawChessboardCorners(colorView, input_image_Boardsize, cv::Mat(pointBuf), found);
		imshow("Successful", colorView);
		cvWaitKey();
#endif
	}
	else
	{
		cv::imshow("Unsuccessful", inputImage);
		cvWaitKey();
	}

	out_image_corners_points = imagePoints;
}

//////////////////////////////////////////////////////////////////////////
//Calibration Part
static void calcBoardCornerPositions(cv::Size boardSize, float squareSize, vector<cv::Point3f>& corners,
	int patternType /*CHESSBOARD*/)
{
	corners.clear();

	switch (patternType)
	{
	case 1:
	case 2:
		for (int i = 0; i < boardSize.height; ++i)
			for (int j = 0; j < boardSize.width; ++j)
				corners.push_back(cv::Point3f(j*squareSize, i*squareSize, 0));
		break;

	case 3:
		for (int i = 0; i < boardSize.height; i++)
			for (int j = 0; j < boardSize.width; j++)
				corners.push_back(cv::Point3f((2 * j + i % 2)*squareSize, i*squareSize, 0));
		break;
	default:
		break;
	}
}

static double computeReprojectionErrors(const vector<vector<cv::Point3f> >& objectPoints,
	const vector<vector<cv::Point2f> >& imagePoints,
	const vector<cv::Mat>& rvecs, const vector<cv::Mat>& tvecs,
	const cv::Mat& cameraMatrix, const cv::Mat& distCoeffs,
	vector<float>& perViewErrors)
{
	vector<cv::Point2f> imagePoints2;
	size_t totalPoints = 0;
	double totalErr = 0, err;
	perViewErrors.resize(objectPoints.size());

	for (size_t i = 0; i < objectPoints.size(); ++i)
	{
		projectPoints(objectPoints[i], rvecs[i], tvecs[i], cameraMatrix, distCoeffs, imagePoints2);
		err = norm(imagePoints[i], imagePoints2, cv::NORM_L2);

		size_t n = objectPoints[i].size();
		perViewErrors[i] = (float)std::sqrt(err*err / n);
		totalErr += err*err;
		totalPoints += n;
	}

	return std::sqrt(totalErr / totalPoints);
}

bool IPHelper_Calibration::RunCalibration(cv::Mat& cameraMatrix, cv::Mat& distCoeffs,
	vector<cv::Mat>& rvecs, vector<cv::Mat>& tvecs,
	vector<float>& reprojErrs, double& totalAvgErr)
{
	cameraMatrix = cv::Mat::zeros(3, 3, CV_64F);
	distCoeffs = cv::Mat::zeros(14, 1, CV_64F);
	//20180213
	//Initialize intrinsic matrix
	cameraMatrix.at<double>(0, 0) = 2900;//fx
	cameraMatrix.at<double>(1, 1) = 2900;//fy
	cameraMatrix.at<double>(2, 2) = 1;//1
	cameraMatrix.at<double>(0, 2) = 320;//cx
	cameraMatrix.at<double>(1, 2) = 256;//cy

	vector<vector<cv::Point3f> > objectPoints(1);
	calcBoardCornerPositions(class_private_image_boardSize, class_private_squareSize, objectPoints[0], class_private_calibration_type);

	vector<vector<cv::Point2f> > imagePoints = class_private_imagePoints;
	objectPoints.resize(imagePoints.size(), objectPoints[0]);

	//Find intrinsic and extrinsic camera parameters
	double rms;

	int flag_calibrate_model = cv::CALIB_USE_INTRINSIC_GUESS + cv::CALIB_TILTED_MODEL;//CALIB_TILTED_MODEL
	rms = calibrateCamera(objectPoints, imagePoints, class_private_imageSize, cameraMatrix, distCoeffs, rvecs, tvecs,flag_calibrate_model);

	cout << "Re-projection error reported by calibrateCamera: " << rms << endl;

	bool ok = checkRange(cameraMatrix) && checkRange(distCoeffs);

	totalAvgErr = computeReprojectionErrors(objectPoints, imagePoints, rvecs, tvecs, cameraMatrix,
		distCoeffs, reprojErrs);

	class_private_cameraMatrix = cameraMatrix;
	class_private_distCoeffs = distCoeffs;
	class_private_rvecs = rvecs;
	class_private_tvecs = tvecs;
	class_private_reprojErrs=reprojErrs;
	class_private_totalAvgErr=totalAvgErr;

	return ok;
}

void IPHelper_Calibration::SaveCameraParams(string file_path, bool save_every_image_detail/* =false */)
{
	cv::FileStorage fs(file_path, cv::FileStorage::WRITE);

	//fs << "calibration_time" << buf;

	if (!class_private_rvecs.empty() || !class_private_reprojErrs.empty())
		fs << "nr_of_frames" << (int)std::max(class_private_rvecs.size(), class_private_reprojErrs.size());
	fs << "image_width" << class_private_imageSize.width;
	fs << "image_height" << class_private_imageSize.height;
	fs << "board_width" << class_private_image_boardSize.width;
	fs << "board_height" << class_private_image_boardSize.height;
	fs << "square_size" << class_private_squareSize;
	fs << "camera_matrix" << class_private_cameraMatrix;
	fs << "distortion_coefficients" << class_private_distCoeffs;
	fs << "avg_reprojection_error" << class_private_totalAvgErr;
	if (save_every_image_detail)
	{
		if (!class_private_reprojErrs.empty())
			fs << "per_view_reprojection_errors" << cv::Mat(class_private_reprojErrs);

		if (!class_private_rvecs.empty() && !class_private_tvecs.empty())
		{
			CV_Assert(class_private_rvecs[0].type() == class_private_tvecs[0].type());
			cv::Mat bigmat((int)class_private_rvecs.size(), 6, CV_MAKETYPE(class_private_rvecs[0].type(), 1));
			bool needReshapeR = class_private_rvecs[0].depth() != 1 ? true : false;
			bool needReshapeT = class_private_tvecs[0].depth() != 1 ? true : false;

			for (size_t i = 0; i < class_private_rvecs.size(); i++)
			{
				cv::Mat r = bigmat(cv::Range(int(i), int(i + 1)), cv::Range(0, 3));
				cv::Mat t = bigmat(cv::Range(int(i), int(i + 1)), cv::Range(3, 6));

				if (needReshapeR)
					class_private_rvecs[i].reshape(1, 1).copyTo(r);
				else
				{
					//*.t() is MatExpr (not Mat) so we can use assignment operator
					CV_Assert(class_private_rvecs[i].rows == 3 && class_private_rvecs[i].cols == 1);
					r = class_private_rvecs[i].t();
				}

				if (needReshapeT)
					class_private_tvecs[i].reshape(1, 1).copyTo(t);
				else
				{
					CV_Assert(class_private_tvecs[i].rows == 3 && class_private_tvecs[i].cols == 1);
					t = class_private_tvecs[i].t();
				}
			}
			fs << "extrinsic_parameters_comment" << "a set of 6-tuples (rotation vector + translation vector) for each view";
			fs << "extrinsic_parameters" << bigmat;
		}
		if (!class_private_imagePoints.empty())
		{
			cv::Mat imagePtMat((int)class_private_imagePoints.size(), (int)class_private_imagePoints[0].size(), CV_32FC2);
			for (size_t i = 0; i < class_private_imagePoints.size(); i++)
			{
				cv::Mat r = imagePtMat.row(int(i)).reshape(2, imagePtMat.cols);
				cv::Mat imgpti(class_private_imagePoints[i]);
				imgpti.copyTo(r);
			}
			fs << "image_points" << imagePtMat;
		}
	}
	fs.release();
}

void IPHelper_Calibration::LoadCameraParams(string xmlCalibrationFilePath, cv::Mat& cameraMatrix, cv::Mat& distCoeffs)
{
	cv::FileStorage fs(xmlCalibrationFilePath, cv::FileStorage::READ);
	cameraMatrix = cv::Mat::eye(3, 3, CV_64F);
	distCoeffs = cv::Mat::zeros(14, 1, CV_64F);
	fs["camera_matrix"] >> cameraMatrix;
	fs["distortion_coefficients"] >> distCoeffs;
}

void IPHelper_Calibration::LoadCameraSize(string xmlCalibrationFilePath, cv::Size& output_BoardSize, double& output_SquareSize)
{
	cv::FileStorage fs(xmlCalibrationFilePath, cv::FileStorage::READ);
	int boardWidth, boardHeight;
	double squareSize;
	fs["board_width"] >> boardWidth;
	fs["board_height"] >> boardHeight;
	fs["square_size"] >> squareSize;
	output_BoardSize = cv::Size(boardWidth, boardHeight);
	output_SquareSize = squareSize;
}