#pragma once
#include <string>
#include <vector>
#include "opencv2/opencv.hpp"
#pragma once

using namespace std;

class FitPlaneHelper
{
public:
	FitPlaneHelper();
	~FitPlaneHelper();
	void getCrossOfLines(cv::Vec4f line1, cv::Vec4f line2, CvPoint2D32f &pt);
	void getFitLineStartEndPt(cv::Mat inputImg, cv::Vec4f line, CvPoint2D32f &pt_start, CvPoint2D32f &pt_end);
	void imgPTtoCameraPT(CvPoint2D32f &imgPT, CvPoint3D32f &cameraPT, cv::Mat cameraIntrinsic, double pixelSize);
	float getAngle(CvPoint3D32f &angle1, CvPoint3D32f &angle2);
	void SVD_Solve(double *SA, double *SU, double *SS, double *SV, int count);
	void getCrossCamera_1(CvPoint3D32f &output_point,float &a1, float &a2, float &a3, float &OP, CvPoint3D32f &cross_point_c, double d);
	void getCrossCamera(CvPoint3D32f &output_point, float &a1, float &a2, float &a3, float &OP, CvPoint3D32f &cross_point_c, double d);
	void PlaneSolve(vector<CvPoint3D32f> &point_c, float *PlaneLight);
	void SavePlaneParameter(string savePath, float *PlaneLight);
	void LoadPlaneParameter(string savePath, float *PlaneLight);
	void Solve_Z_With_Plane(CvPoint3D32f &ray_point, CvPoint2D32f &pt, cv::Mat &cameraIntrinsic, double pixelSize, float* planeLight);
	void Solve_Z_With_Plane2(CvPoint3D32f &ray_point, CvPoint2D32f &pt, cv::Mat &cameraIntrinsic, double pixelSize, float* planeLight);
	void PlaneTest(vector<CvPoint3D32f> &output_testline, float *LightPlane, cv::Mat &cameraIntrinsic,CvPoint2D32f testPoint);
protected:
private:
};