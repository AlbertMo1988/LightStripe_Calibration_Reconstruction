#include "FitPlaneHelper.h"
#include <io.h>
#include <stdlib.h>
#include <fstream>
#include <iostream>
#include <string>
#include <sstream>

using namespace std;

FitPlaneHelper::FitPlaneHelper(){}
FitPlaneHelper::~FitPlaneHelper(){}

void FitPlaneHelper::getCrossOfLines(cv::Vec4f line1, cv::Vec4f line2, CvPoint2D32f &pt)
{
	//求取角点线和结构光中心线的交点
	if (line1[0] == 0)
		line1[0] = 0.0001f;
	if (line2[0] == 0)
		line2[0] = 0.0001f;
	float k1 = line1[1] / line1[0];
	float b1 = line1[3] - k1*line1[2];
	float k2 = line2[1] / line2[0];
	float b2 = line2[3] - k2*line2[2];
	float deltaK = k1 - k2;
	if (deltaK == 0)
		deltaK = 0.0001f;

	pt.x = (b2 - b1) / deltaK;
	pt.y = k1*(pt.x) + b1;
}

void FitPlaneHelper::getFitLineStartEndPt(cv::Mat inputImg, cv::Vec4f line, CvPoint2D32f &pt_start, CvPoint2D32f &pt_end)
{
	CvPoint2D32f startpt;
	CvPoint2D32f endpt;
	float t = (float)(inputImg.cols + inputImg.rows);
	startpt.x = cvRound(line[2] - line[0] * t);
	startpt.y = cvRound(line[3] - line[1] * t);
	endpt.x = cvRound(line[2] + line[0] * t);
	endpt.y = cvRound(line[3] + line[1] * t);
	pt_start = startpt;
	pt_end = endpt;
}

void FitPlaneHelper::imgPTtoCameraPT(CvPoint2D32f &imgPT, CvPoint3D32f &cameraPT, cv::Mat cameraIntrinsic, double pixelSize)
{
	cameraPT.x = (imgPT.x - cameraIntrinsic.at<double>(0,2)) * pixelSize;
	cameraPT.y = (imgPT.y - cameraIntrinsic.at<double>(1,2)) * pixelSize; //此处有修改？？？？cameraPT.y = (CV_MAT_ELEM(*intrinsic, float, 1, 2) - imgPT.y) * dy; 
	cameraPT.z = cameraIntrinsic.at<double>(0, 0)*pixelSize;//Focus
}

float FitPlaneHelper::getAngle(CvPoint3D32f &angle1, CvPoint3D32f &angle2)
{
	//求两点之间夹角
	float dot_product = angle1.x*angle2.x + angle1.y*angle2.y + angle1.z*angle2.z; //向量点乘
	float module1 = sqrt(angle1.x*angle1.x + angle1.y*angle1.y + angle1.z*angle1.z);
	float module2 = sqrt(angle2.x*angle2.x + angle2.y*angle2.y + angle2.z*angle2.z);
	float angle = acos(dot_product / (module1*module2)); //返回弧度制角度
	return angle;
}

void FitPlaneHelper::SVD_Solve(double *SA, double *SU, double *SS, double *SV, int count)
{
	//根据多个点拟合光平面
	//解奇异方程组 AX+BY+CZ+D=0
	CvMat A, U, S, V;
	cvInitMatHeader(&A, count, 4, CV_64FC1, SA, CV_AUTOSTEP);
	cvInitMatHeader(&U, 4, 4, CV_64FC1, SU, CV_AUTOSTEP);       //固定分解成4X4的矩阵。
	cvInitMatHeader(&S, 4, 4, CV_64FC1, SS, CV_AUTOSTEP);
	cvInitMatHeader(&V, 4, 4, CV_64FC1, SV, CV_AUTOSTEP);
	cvSVD(&A, &U, &S, &V, CV_SVD_U_T);
}

void FitPlaneHelper::getCrossCamera_1(CvPoint3D32f &output_point, float &a1, float &a2, float &a3, float &OP, CvPoint3D32f &cross_point_c, double squareSize)
{
	//获取交点P的摄像机坐标
	if (a1 > 1.57)
	{
		cout << "角度大于90度!重新摆放位置或编写程序" << endl;
		system("Pause");
		return;
	}
	else
	{
		double ta1 = tan(a1);
		double ta2 = tan(a2);  //要保证各个角度比较小，小于90度，否则tans可能不对了,斜率要重新改
		double ta3 = tan(a3);
		//cout<<"a3------------:"<<a3<<endl;
		//自己写的求取OP距离的函数
		float xb = sqrt(squareSize*squareSize / (4 * ta2*ta2 / (ta1*ta1) - 4 * ta2 / ta1 + 1 + ta2*ta2));//可能有bug，棋盘格物理尺寸？？？
		float xa = 2 * ta2*xb / ta1;
		float k = (ta1*xa - ta2*xb) / (xa - xb);
		float x = ta2*xb - k *xb / (ta3 - k);
		OP = sqrt(x*x + ta3*x*ta3*x);
	}
	cout << "OP-------------:" << OP << endl;
	CvPoint3D32f temp_point;
	float temp_module = sqrt(cross_point_c.x*cross_point_c.x + cross_point_c.y*cross_point_c.y + cross_point_c.z*cross_point_c.z);
	temp_point.x = 10 * OP * (cross_point_c.x / temp_module);
	temp_point.y = 10 * OP * (cross_point_c.y / temp_module);
	temp_point.z = 10 * OP * (cross_point_c.z / temp_module); //单位是 mm
	//point_c.push_back(temp_point);//将每个点的摄像机坐标保存到vector中
	output_point = temp_point;
}

void FitPlaneHelper::getCrossCamera(CvPoint3D32f &output_point, float &a1, float &a2, float &a3, float &OP, CvPoint3D32f &cross_point_c, double d)
{
	if (a1 > 1.57)
	{
		cout << "角度大于90度!重新摆放位置或编写程序" << endl;
		system("Pause");
		return;
	}
	else
	{
		double ta1 = tan(a1);
		double ta2 = tan(a2);  //要保证各个角度比较小，小于90度，否则tans可能不对了,斜率要重新改
		double ta3 = tan(a3);  //a3: p和l3交点
		if ((abs(ta2-(2*ta1)))<0.001)//l与y轴平行 2ta1=ta2
		{
			OP = d / (ta1*cos(a3));
		}
		else
		{
			double k = ta1*ta2 / (2 * ta1 - ta2);
			double b = d*k*(ta1 - k) / (sqrt(1 + k*k)*ta1);
			OP = b / ((ta3 - k)*cos(a3));
		}
	}
	cout << "OP-------------:" << OP << endl;
	CvPoint3D32f temp_point;
	float temp_module = sqrt(cross_point_c.x*cross_point_c.x + cross_point_c.y*cross_point_c.y + cross_point_c.z*cross_point_c.z);
	temp_point.x = OP * (cross_point_c.x / temp_module);
	temp_point.y = OP * (cross_point_c.y / temp_module);
	temp_point.z = OP * (cross_point_c.z / temp_module); //单位是 mm
	//point_c.push_back(temp_point);//将每个点的摄像机坐标保存到vector中
	output_point = temp_point;
}

void FitPlaneHelper::PlaneSolve(vector<CvPoint3D32f> &point_c, float *PlaneLight)
{
	//求解结构光平面：根据解奇异方程组求解光平面系数 AX+ BY+ CZ+ D=0 。

	int lenth = point_c.size();
	cout << lenth <<endl;
	if (lenth < 4)
	{
		cout << "光条交点数量过少!" << endl;
		system("Pause");
		return;
	}
	double* A = (double*)malloc(sizeof(double) * 4 * lenth);
	for (int i = 0; i<lenth; i++)
	{
		CvPoint3D32f temp = point_c[i];
		A[4 * i] = (double)temp.x;
		A[4 * i + 1] = (double)temp.y;
		A[4 * i + 2] = (double)temp.z;
		A[4 * i + 3] = 1;
	}
	double U[16], S[16], V[16];
	SVD_Solve(A, U, S, V, lenth);
	PlaneLight[0] = V[3] / V[3];   //系数A
	PlaneLight[1] = V[7] / V[3];   //系数B
	PlaneLight[2] = V[11] / V[3];  //系数C
	PlaneLight[3] = V[15] / V[3];  //系数D
	cout << "A:" << PlaneLight[0] << endl;
	cout << "B:" << PlaneLight[1] << endl;
	cout << "C:" << PlaneLight[2] << endl;
	cout << "D:" << PlaneLight[3] << endl;
}

void FitPlaneHelper::SavePlaneParameter(string savePath, float *PlaneLight)
{
	cv::FileStorage fs(savePath, cv::FileStorage::WRITE);
	fs << "A" << PlaneLight[0];
	fs << "B" << PlaneLight[1];
	fs << "C" << PlaneLight[2];
	fs << "D" << PlaneLight[3];
}

void FitPlaneHelper::LoadPlaneParameter(string savePath, float *PlaneLight)
{
	cv::FileStorage fs(savePath, cv::FileStorage::READ);
	fs["A"] >> PlaneLight[0];
	fs["B"] >> PlaneLight[1];
	fs["C"] >> PlaneLight[2];
	fs["D"] >> PlaneLight[3];
}

void FitPlaneHelper::Solve_Z_With_Plane(CvPoint3D32f &ray_point, CvPoint2D32f &pt, cv::Mat &cameraIntrinsic, double pixelSize, float* planeLight)
{
	double u0 = cameraIntrinsic.at<double>(0, 2);
	double v0 = cameraIntrinsic.at<double>(1, 2);
	double focus = cameraIntrinsic.at<double>(0, 0)*pixelSize;//Focus
	double x_c = (pt.x - u0)*pixelSize;
	double y_c = (pt.y - v0)*pixelSize;
	//X=(x_c/f)*Z, Y=(y_c/f)*Z, Ax+By+Cz+D=0
	float A = planeLight[0];
	float B = planeLight[1];
	float C = planeLight[2];
	float D = planeLight[3];
	double parm_x = A*(x_c / focus);
	double parm_y = B*(y_c / focus);
	double t = (-D) / (parm_x + parm_y + C);
	ray_point = cv::Point3d(parm_x*t, parm_y*t, t);
}

void FitPlaneHelper::Solve_Z_With_Plane2(CvPoint3D32f &ray_point, CvPoint2D32f &pt, cv::Mat &cameraIntrinsic, double pixelSize, float* planeLight)
{
	double u0 = cameraIntrinsic.at<double>(0, 2);
	double v0 = cameraIntrinsic.at<double>(1, 2);
	double focus = cameraIntrinsic.at<double>(0, 0)*pixelSize;//Focus
	double x_u = (pt.x - u0)*pixelSize;
	double y_v = (pt.y - v0)*pixelSize;
	//X=(x_c/f)*Z, Y=(y_c/f)*Z, Ax+By+Cz+D=0
	float A = planeLight[0];
	float B = planeLight[1];
	float C = planeLight[2];
	float D = planeLight[3];
	double x_c = D*x_u / (A*x_u + B*y_v + focus*C);
	double y_c = D*y_v / (A*x_u + B*y_v + focus*C);
	double z_c = -focus*D / (A*x_u + B*y_v + focus*C);
	ray_point = cv::Point3d(x_c, y_c, z_c);
}

void FitPlaneHelper::PlaneTest(vector<CvPoint3D32f> &output_testline, float *LightPlane, cv::Mat &cameraIntrinsic, CvPoint2D32f testPoint)
{
	double a[16];
	float A = LightPlane[0];
	float B = LightPlane[1];
	float C = LightPlane[2];
	float D = LightPlane[3];
	double fx = cameraIntrinsic.at<double>(0, 0);
	double fy = cameraIntrinsic.at<double>(1, 1);
	double cx = cameraIntrinsic.at<double>(0, 2);
	double cy = cameraIntrinsic.at<double>(1, 2);
	double u = testPoint.x;
	double v = testPoint.y;
	a[0] = A; a[1] = B; a[2] = C; a[3] = D;
	a[4] = 0; a[5] = fy; a[6] = v - cy; a[7] = 0;
	a[8] = fx; a[9] = 0; a[10] = cx - u; a[11] = 0;
	a[12] = 0; a[13] = 0; a[14] = 0; a[15] = 0;
	double U[16], S[16], V[16];
	SVD_Solve(a, U, S, V, 4);
	CvPoint3D32f temp;
	temp.x = V[3] / V[15];   //系数A
	temp.y = V[7] / V[15];   //系数B
	temp.z = V[11] / V[15];  //系数C
	output_testline.push_back(temp);
}