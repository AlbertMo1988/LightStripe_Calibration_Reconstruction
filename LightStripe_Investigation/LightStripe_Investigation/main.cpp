// LightStripe_Investigation.cpp: Main
//
#include <opencv2/opencv.hpp>
#include <string.h>
#include "IPHelper.h"
#include "IPHelper_Calibration.h"
#include "FileHelper.h"
#include "FitPlaneHelper.h"

using namespace std;

void Calibration_Circle(string imageFolder, string xmlSavePath, int boardSize, double squareSize, string cornersSavePath = "");
void Calibration_Intrinsic();
void Calibration_LightPlane();
void Reconstruct();

int main()
{
	int model = 1;// 0: Camera Calibrtion; 1: LightPlane Calibration, 2: Reconstruct
	if (model == 0)
	{
		Calibration_Intrinsic();
	}
	if (model == 1)
	{
		Calibration_LightPlane();
	}
	if (model == 2)
	{
		Reconstruct();
	}
	return 0;
}

void Calibration_Circle(string imageFolder, string xmlSavePath, int boardSize, double squareSize, string cornersSavePath /* = "" */)
{
	IPHelper_Calibration ipHelper_Calibration(cvSize(boardSize, boardSize), squareSize, 2);
	vector<string> image_paths;
	ipHelper_Calibration.Load_Single_Camera_Image_Path(image_paths, imageFolder);
	vector<vector<cv::Point2f>> corners;
	ipHelper_Calibration.Find_Single_Camera_Image_Corners(corners);
	cv::Mat cameraMatrix, distCoeffs;
	vector<cv::Mat> rvecs, tvecs;
	vector<float> reprojErrs;
	double totalAvgErr;
	bool calibrate_success = ipHelper_Calibration.RunCalibration(cameraMatrix, distCoeffs, rvecs, tvecs, reprojErrs, totalAvgErr);
	cout << (calibrate_success ? "Calibration succeeded" : "Calibration failed")
		<< ". avg re projection error = " << totalAvgErr << endl;
	ipHelper_Calibration.SaveCameraParams(xmlSavePath, false);
	cv::waitKey();
}

void Calibration_Intrinsic()
{
	cout << "Input 2 arguments: Image folder path, Save calibration result path" << endl;
	char char_folderPath[200];
	char char_saveFilePath[200];
	cin >> char_folderPath;
	cin >> char_saveFilePath;
	string folderPath(char_folderPath);
	string saveFilePath(char_saveFilePath);
	folderPath = "E:\\Test_Images\\" + folderPath;
	saveFilePath = "E:\\Test_Images\\" + saveFilePath;
	cout << "Input 2 arguments: boardSize, squareSize(mm):" << endl;
	int boardSize;
	double squareSize;
	cin >> boardSize;
	cin >> squareSize;
	Calibration_Circle(folderPath, saveFilePath, boardSize, squareSize);
	system("Pause");
}

void Calibration_LightPlane()
{
	float planeLight[4];                             //光平面。
	vector<CvPoint3D32f> point_c;					//存储所有图像的交点的摄像机坐标
	vector<CvPoint3D32f> testline;

	//Remap and fit light plane
	cout << "Input calibration file path (E:\\Test_Images\\...)" << endl;
	char char_Path[200];
	cin >> char_Path;
	string calibration_file_path(char_Path);
	calibration_file_path = "E:\\Test_Images\\" + calibration_file_path;
	cout << "Input the number of image pairs need to fit light plane" << endl;
	int num_image_pairs;
	cin >> num_image_pairs;

	cv::Mat cameraMatrix, cameraDistortion;
	cv::Size BoardSize;
	double PixelSize = 4.8 / 1000;//mm //固定像元大小，调整Focus;另一种是固定Focus，调整dx,dy大小
	double SquareSize;
	//Every line has more than 3 points
	IPHelper_Calibration class_IPHelper_Calibration;
	class_IPHelper_Calibration.LoadCameraParams(calibration_file_path, cameraMatrix, cameraDistortion);
	class_IPHelper_Calibration.LoadCameraSize(calibration_file_path, BoardSize, SquareSize);
	int step_corner = BoardSize.height / 3;
	FitPlaneHelper class_FitPlaneHelper;
	IPHelper class_IPHelper;

	for (int i = 0; i < num_image_pairs; i++)
	{
		//每张图作处理
		cv::Mat img_calibration_plate, img_laser;
		cv::Mat img_Demo;

		CvPoint3D32f cross_point_c;										//存储单线交点p摄像机坐标
		float* OP = new float(BoardSize.width);							//存储每幅图像的交点摄像机坐标的|OP|值
		cv::Vec4f laser_line_fit;										//结构光直线拟合结果
		vector<vector<cv::Point2f>> corners_y_x;						//检测到的角点在图像上的坐标，<每列<每行>>
		vector<cv::Vec4f> corner_line_fit;								//拟合焦点线方程，每张图有width个角点线方程
		float a1, a2, a3;

		cout << "Input calibration plate path (E:\\Test_Images\\...)" << endl;
		cin >> char_Path;
		string img_Path_1(char_Path);
		string img_Path = "E:\\Test_Images\\" + img_Path_1;
		img_calibration_plate = cv::imread(img_Path, cv::IMREAD_GRAYSCALE);

		cout << "Input laser image path (E:\\Test_Images\\...)" << endl;
		cin >> char_Path;
		string img_Path_2(char_Path);
		img_Path = "E:\\Test_Images\\" + img_Path_2;
		img_laser = cv::imread(img_Path, cv::IMREAD_GRAYSCALE);

		if (img_laser.empty() || img_calibration_plate.empty())
		{
			cout << "Image data is wrong!!" << endl;
			system("Pause");
			i--;
			continue;
		}

		//Start remap
		cv::Mat img_undistort_calibration_plate, img_undistort_laser;
		class_IPHelper_Calibration.UnDistortImage(img_calibration_plate, img_undistort_calibration_plate, cameraMatrix, cameraDistortion);
		class_IPHelper_Calibration.UnDistortImage(img_laser, img_undistort_laser, cameraMatrix, cameraDistortion);

#ifdef _DEBUG
		cv::imshow("Calibration_Plate_Origin", img_calibration_plate);
		cv::imshow("Calibration_Plate_Undistort", img_undistort_calibration_plate);
		cv::imshow("Laser_Origin", img_laser);
		cv::imshow("Laser_Undistort", img_undistort_laser);
		cvWaitKey(0);
#endif

		//Fit laser line-->laser_line_fit
		vector<cv::Point> laser_points;
		class_IPHelper.Extract_Points_Best_Upper(img_undistort_laser, laser_points, 0);//Check !!!
		cv::fitLine(laser_points, laser_line_fit, CV_DIST_L2, 0, 0.01, 0.01);

#ifdef _DEBUG
		class_IPHelper.CVT_Gray2RGB(img_undistort_laser, img_Demo);
		CvPoint2D32f startpt;
		CvPoint2D32f endpt;
		class_FitPlaneHelper.getFitLineStartEndPt(img_Demo, laser_line_fit, startpt, endpt);
		line(img_Demo, cvPoint(startpt.x, startpt.y), cvPoint(endpt.x, endpt.y), cv::Scalar(255, 255, 255), 5);
		imshow("Demo", img_Demo);
		cvWaitKey(0);
#endif
		vector<vector<cv::Point2f>> corners;
		class_IPHelper_Calibration.Find_Single_Camera_Image_Corners_Standalone(BoardSize, 2, img_undistort_calibration_plate, corners);
		vector<cv::Point2f> corners_temp = corners[0];
		for (int j = 0; j < BoardSize.width; j++)
		{
			vector<cv::Point2f> corners_temp_one_line_y;
			for (int k = 0; k < BoardSize.height; k++)
			{
				corners_temp_one_line_y.push_back(corners_temp[j + BoardSize.width*k]);
			}
			corners_y_x.push_back(corners_temp_one_line_y);
		}
		//Now the corners are stored every vertical line (x line)
		//Fit every corner line
		for (int j = 0; j < BoardSize.width; j++)
		{
			corners_temp = corners_y_x[j];//jth line
			cv::Vec4f corner_line_fit_temp;
			cv::fitLine(corners_temp, corner_line_fit_temp, CV_DIST_L2, 0, 0.01, 0.01);
#ifdef _DEBUG
			class_IPHelper.CVT_Gray2RGB(img_undistort_calibration_plate, img_Demo);
			class_FitPlaneHelper.getFitLineStartEndPt(img_Demo, corner_line_fit_temp, startpt, endpt);
			line(img_Demo, cvPoint(startpt.x, startpt.y), cvPoint(endpt.x, endpt.y), cv::Scalar(0, 255, 0), 5);
			imshow("Demo", img_Demo);
			cvWaitKey(0);
#endif
			corner_line_fit.push_back(corner_line_fit_temp);
		}

		//Now calculate every corner line per image
		for (int j = 0; j < BoardSize.width; j++)
		{
			corners_temp = corners_y_x[j];//jth line
										  //分别获得中心线和角点线的交点投影p的像素坐标
			CvPoint2D32f cross_point;
			class_FitPlaneHelper.getCrossOfLines(laser_line_fit, corner_line_fit[j], cross_point);
			//planeTest(cross_point);
#ifdef _DEBUG
			class_IPHelper.CVT_Gray2RGB(img_undistort_calibration_plate, img_Demo);
			circle(img_Demo, cross_point, 6, cv::Scalar(0, 255, 255));
			imshow("Demo", img_Demo);
			cvWaitKey(0);
#endif
			//Transfer p to camera coordinate
			class_FitPlaneHelper.imgPTtoCameraPT(cross_point, cross_point_c, cameraMatrix, PixelSize);
			//Get 3 corners and transfer to camera coordinate
			//a,b,c,p--> 0,1,2
			vector<CvPoint3D32f> image_points_buf_c;			//存储单线角点摄像机坐标
			for (int k = 0; k < 3; k++)
			{
				CvPoint3D32f temp_3d_point;
				CvPoint2D32f temp_2d_point;
				temp_2d_point.x = corners_temp[k*step_corner].x;
				temp_2d_point.y = corners_temp[k*step_corner].y;
				class_FitPlaneHelper.imgPTtoCameraPT(temp_2d_point, temp_3d_point, cameraMatrix, PixelSize);
				image_points_buf_c.push_back(temp_3d_point);
			}
			//Get Angle
			a1 = class_FitPlaneHelper.getAngle(image_points_buf_c[0], image_points_buf_c[1]);//Oc,Ob
			a2 = class_FitPlaneHelper.getAngle(image_points_buf_c[0], image_points_buf_c[2]);//Oc,Oa
			a3 = class_FitPlaneHelper.getAngle(image_points_buf_c[0], cross_point_c);//Oc,Op
			CvPoint3D32f cross_point_3d_c_temp;
			class_FitPlaneHelper.getCrossCamera2(cross_point_3d_c_temp, a1, a2, a3, OP[j], cross_point_c, SquareSize*step_corner);
			point_c.push_back(cross_point_3d_c_temp);
		}
	}
	class_FitPlaneHelper.PlaneSolve(point_c, planeLight);
	cout << "Input lightplane save path (E:\\Test_Images\\...)" << endl;
	cin >> char_Path;
	string lightplane_path(char_Path);
	lightplane_path = "E:\\Test_Images\\" + lightplane_path;
	class_FitPlaneHelper.SavePlaneParameter(lightplane_path, planeLight);
}

void Reconstruct()
{
	//Remap
	cout << "Input calibration file path (E:\\Test_Images\\...)" << endl;
	char char_Path[200];
	cin >> char_Path;
	string calibration_file_path(char_Path);
	calibration_file_path = "E:\\Test_Images\\" + calibration_file_path;
	cv::Mat cameraMatrix, cameraDistortion;
	double PixelSize = 4.8 / 1000;//mm //固定像元大小，调整Focus;另一种是固定Focus，调整dx,dy大小
	IPHelper_Calibration class_IPHelper_Calibration;
	class_IPHelper_Calibration.LoadCameraParams(calibration_file_path, cameraMatrix, cameraDistortion);

	FitPlaneHelper class_FitPlaneHelper;
	float planeLight[4];
	cout << "Input lightplane file path (E:\\Test_Images\\...)" << endl;
	cin >> char_Path;
	string planelight_file_path(char_Path);
	planelight_file_path = "E:\\Test_Images\\" + planelight_file_path;
	class_FitPlaneHelper.LoadPlaneParameter(planelight_file_path, planeLight);

	IPHelper class_IPHelper;
	bool flag = true;
	while (flag)
	{
		//Start to Extract points
		cout << "Input image file path (E:\\Test_Images\\...)" << endl;
		cin >> char_Path;
		string image_file_path(char_Path);
		image_file_path = "E:\\Test_Images\\" + image_file_path;
		cv::Mat img_ori = cv::imread(image_file_path, CV_LOAD_IMAGE_GRAYSCALE);
		cv::Mat img_undistort, img_Demo;
		if (img_ori.empty())
		{
			cout << "Image data is wrong!!" << endl;
			system("Pause");
			return;
		}

		class_IPHelper_Calibration.UnDistortImage(img_ori, img_undistort, cameraMatrix, cameraDistortion);
		vector<cv::Point> extracted_points;
		class_IPHelper.Extract_Points_Best_Upper(img_undistort, extracted_points, 50);
		vector<double> reconstructured_Z_demo, reconstructured_X_demo;
		vector<CvPoint3D32f> reconstructured_points;
		for (int i = 0; i < extracted_points.size(); i++)
		{
			CvPoint2D32f pt_temp = extracted_points[i];
			if (pt_temp.y != 0)
			{
				CvPoint3D32f temp_p;
				class_FitPlaneHelper.Solve_Z_With_Plane2(temp_p, pt_temp, cameraMatrix, PixelSize, planeLight);
				reconstructured_Z_demo.push_back(temp_p.z);
				reconstructured_X_demo.push_back(pt_temp.x);
				reconstructured_points.push_back(temp_p);
			}
		}
		//Demo
		class_IPHelper.CVT_Gray2RGB(img_undistort, img_Demo);
		for (int i = 0; i < reconstructured_X_demo.size(); i++)
		{
			double z_value = reconstructured_Z_demo[i];
			double x = reconstructured_X_demo[i];
			img_Demo.at<cv::Vec3b>(z_value, x)[0] = 255;
		}
		for (int i = 0; i < extracted_points.size(); i++)
		{
			CvPoint2D32f pt_temp = extracted_points[i];
			if (pt_temp.y != 0)
			{
				circle(img_Demo, pt_temp, 1, cv::Scalar(0, 255, 0), -1);
			}
		}
		imshow("Demo", img_Demo);
		cvWaitKey(0);

		cout << "Continue?" << endl;
		cin >> char_Path;
		string strFlag(char_Path);
		if (strFlag != "Y")
		{
			flag = false;
		}
	}
}

