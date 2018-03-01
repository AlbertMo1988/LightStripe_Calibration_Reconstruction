#include "IPHelper.h"
#include <io.h>
#include <stdlib.h>
#include <fstream>
#include <iostream>
#include <string>
#include <sstream>

using namespace std;

IPHelper::IPHelper()
{
}

IPHelper::~IPHelper()
{
}

int getThresh_twoDimensionOTSU(const cv::Mat &image)
{
	{
		int height = image.rows;
		int width = image.cols;
		double dHistogram[256][256];        //建立二维灰度直方图  
		double dTrMatrix = 0.0;             //离散矩阵的迹  
		int N = height*width;               //总像素数  
		int i;
		for (i = 0; i < 256; i++)
		{
			for (int j = 0; j < 256; j++)
				dHistogram[i][j] = 0.0;      //初始化变量  
		}
		for (i = 0; i < height; i++)
		{
			for (int j = 0; j < width; j++)
			{
				//unsigned char nData1 = (unsigned char)cvmGet(&pGrayMat, i, j);  
				uchar nData1 = image.at<uchar>(i, j);//当前的灰度值  
				unsigned char nData2 = 0;
				int nData3 = 0;         //注意9个值相加可能超过一个字节  
				for (int m = i - 1; m <= i + 1; m++)
				{
					for (int n = j - 1; n <= j + 1; n++)
					{
						if ((m >= 0) && (m < height) && (n >= 0) && (n < width))
							//nData3 += (unsigned char)cvmGet(&pGrayMat, m, n); //当前的灰度值  
							nData3 += image.at<uchar>(m, n);
					}
				}
				nData2 = (unsigned char)(nData3 / 9);    //对于越界的索引值进行补零,邻域均值  
				dHistogram[nData1][nData2]++;
			}
		}
		for (i = 0; i < 256; i++)
			for (int j = 0; j < 256; j++)
				dHistogram[i][j] /= N;  //得到归一化的概率分布  

		double Pai = 0.0;      //目标区均值矢量i分量  
		double Paj = 0.0;      //目标区均值矢量j分量  
		double Pbi = 0.0;      //背景区均值矢量i分量  
		double Pbj = 0.0;      //背景区均值矢量j分量  
		double Pti = 0.0;      //全局均值矢量i分量  
		double Ptj = 0.0;      //全局均值矢量j分量  
		double W0 = 0.0;       //目标区的联合概率密度  
		double W1 = 0.0;       //背景区的联合概率密度  
		double dData1 = 0.0;
		double dData2 = 0.0;
		double dData3 = 0.0;
		double dData4 = 0.0;   //中间变量  
		int nThreshold_s = 0;
		int nThreshold_t = 0;
		double temp = 0.0;     //寻求最大值  
		for (i = 0; i < 256; i++)
		{
			for (int j = 0; j < 256; j++)
			{
				Pti += i*dHistogram[i][j];
				Ptj += j*dHistogram[i][j];
			}
		}
		for (i = 0; i < 256; i++)
		{
			for (int j = 0; j < 256; j++)
			{
				W0 += dHistogram[i][j];
				dData1 += i*dHistogram[i][j];
				dData2 += j*dHistogram[i][j];

				W1 = 1 - W0;
				dData3 = Pti - dData1;
				dData4 = Ptj - dData2;

				Pai = dData1 / W0;
				Paj = dData2 / W0;
				Pbi = dData3 / W1;
				Pbj = dData4 / W1;   // 得到两个均值向量，用4个分量表示  
				dTrMatrix = ((W0*Pti - dData1)*(W0*Pti - dData1) + (W0*Ptj - dData1)*(W0*Ptj - dData2)) / (W0*W1);
				if (dTrMatrix > temp)
				{
					temp = dTrMatrix;
					nThreshold_s = i;
					nThreshold_t = j;
				}
			}
		}
		int nThreshold = nThreshold_t;   //返回结果中的灰度值  
		return nThreshold;
	}

}

void IPHelper::CVT_RGB2Gray(cv::Mat& src, cv::Mat& dst)
{
	if (src.channels()==1)
	{
		dst = src;
		return;
	}
	else
	{
		cvtColor(src, dst, CV_BGR2GRAY);
		return;
	}
}

void IPHelper::CVT_Gray2RGB(cv::Mat& src, cv::Mat& dst)
{
	if (src.channels()==3)
	{
		dst = src;
		return;
	}
	else
	{
		cvtColor(src, dst, CV_GRAY2BGR);
		return;
	}
}

void IPHelper::Seg_OTSU(cv::Mat& src, cv::Mat& dst, int OTSU_threshold_percentage/* = 100*/)
{
	if (src.channels()!=1)
	{
		return;
	}
	if (src.depth()!=CV_8U)
	{
		return;
	}
	if (OTSU_threshold_percentage==100)
	{
		threshold(src, dst, 0, 255, CV_THRESH_BINARY | CV_THRESH_OTSU);
	}
	else
	{
		int OTSU_threshold = getThresh_twoDimensionOTSU(src);
		threshold(src, dst, OTSU_threshold*OTSU_threshold_percentage/100, 255, CV_THRESH_BINARY);
	}
	return;
}

void IPHelper::Edge_Detection(cv::Mat& src, cv::Mat& dst, int model, int par1, int par2, int par3)
{
	cv::Mat imgSobel_16S;
	switch (model)
	{
	case 1://Sobel
		//par1: Sobel_XY. 1-->X;2-->Y;3-->XY
		//par2: Size
		//par3: threshold
		if (par1==3)
		{
			cv::Mat imgSobelX, imgSobelY;
			Sobel(src, imgSobelX, CV_16S, 1, 0, par2);
			Sobel(src, imgSobelY, CV_16S, 0, 1, par2);
			imgSobel_16S = abs(imgSobelX) + abs(imgSobelY);
		}
		else
		{
			if (par1==1)
			{
				Sobel(src, imgSobel_16S, CV_16S, 1, 0, par2);
			}
			else
			{
				Sobel(src, imgSobel_16S, CV_16S, 0, 1, par2);
			}
		}
		double minVal, maxVal;
		minMaxLoc(imgSobel_16S, &minVal, &maxVal);
		imgSobel_16S.convertTo(dst, CV_8U, 255.0 / (maxVal - minVal), -minVal * 255.0 / (maxVal - minVal));
		break;
	case 2://Canny
		Canny(src, dst, par1, par2, par3);
		break;
	}
}

void IPHelper::Hough_Line(cv::Mat& src, vector<cv::Point>& startPoint, vector<cv::Point>& endPoint)
{
	vector<cv::Vec2f> lines;
	HoughLines(src, lines, 1, CV_PI / 180, 150, 0, 0);
	
	int i = 0;
	int lines_number = lines.size();
	if (lines_number==0)
	{
		return;
	}
	else
	{
		for (i = 0; i < lines_number;i++)
		{
			float rho = lines[i][0], theta = lines[i][1];
			cv::Point pt1, pt2;
			double a = cos(theta), b = sin(theta);
			double x0 = a*rho, y0 = b*rho;
			pt1.x = cvRound(x0 + 1000 * (-b));
			pt1.y = cvRound(y0 + 1000 * (a));
			pt2.x = cvRound(x0 - 1000 * (-b));
			pt2.y = cvRound(y0 - 1000 * (a));
			startPoint.push_back(pt1);
			endPoint.push_back(pt2);
			//line(dstImage, pt1, pt2, Scalar(55, 100, 195), 1, CV_AA);
		}
	}
}

void IPHelper::Hough_LineP(cv::Mat& src, vector<cv::Point>& startPoint, vector<cv::Point>& endPoint, 
	double rho /* = 1 */, double theta /* = CV_PI / 180 */, int threshold /* = 80 */, double minLineLength /* = 0 */, double maxLineGap /* = 0 */)
{
	/*
	Hough Line with Probabilistic is an optimizaiton of Hough Transform. It reduce the computation cost.
	rho: Distance resolution of the accumulator in pixels;
	theta: Angle resolution of the accumulator in radians;
	threshold: Accumulator therehold. Only thise lines are returned that get enough votes(>threhsold);
	minLineLength: Minimum lines length;
	MaxLineGap: Maximum allowed gap between points on the same line to link them.
	*/
	//////////////////////////////////////////////////////////////////////////
	vector<cv::Vec4i> lines;
	HoughLinesP(src, lines, rho, theta, threshold, minLineLength, maxLineGap);
	size_t i = 0;
	size_t linesNum = lines.size();
	for (i = 0; i < linesNum;i++)
	{
		startPoint.push_back(cv::Point(lines[i][0],lines[i][1]));
		endPoint.push_back(cv::Point(lines[i][2], lines[i][3]));
	}
}

void IPHelper::Skeleton(const cv::Mat &src, cv::Mat &dst, const int iterations)
{
	const int height = src.rows - 1;
	const int width = src.cols - 1;

	//拷贝一个数组给另一个数组
	if (src.data != dst.data)
	{
		src.copyTo(dst);
	}


	int n = 0, i = 0, j = 0;
	cv::Mat tmpImg;
	uchar *pU, *pC, *pD;
	bool isFinished = false;

	for (n = 0; n < iterations; n++)
	{
		dst.copyTo(tmpImg);
		isFinished = false;   //一次 先行后列扫描 开始
		//扫描过程一 开始
		for (i = 1; i < height; i++)
		{
			pU = tmpImg.ptr<uchar>(i - 1);
			pC = tmpImg.ptr<uchar>(i);
			pD = tmpImg.ptr<uchar>(i + 1);
			for (int j = 1; j < width; j++)
			{
				if (pC[j] > 0)
				{
					int ap = 0;
					int p2 = (pU[j] > 0);
					int p3 = (pU[j + 1] > 0);
					if (p2 == 0 && p3 == 1)
					{
						ap++;
					}
					int p4 = (pC[j + 1] > 0);
					if (p3 == 0 && p4 == 1)
					{
						ap++;
					}
					int p5 = (pD[j + 1] > 0);
					if (p4 == 0 && p5 == 1)
					{
						ap++;
					}
					int p6 = (pD[j] > 0);
					if (p5 == 0 && p6 == 1)
					{
						ap++;
					}
					int p7 = (pD[j - 1] > 0);
					if (p6 == 0 && p7 == 1)
					{
						ap++;
					}
					int p8 = (pC[j - 1] > 0);
					if (p7 == 0 && p8 == 1)
					{
						ap++;
					}
					int p9 = (pU[j - 1] > 0);
					if (p8 == 0 && p9 == 1)
					{
						ap++;
					}
					if (p9 == 0 && p2 == 1)
					{
						ap++;
					}
					if ((p2 + p3 + p4 + p5 + p6 + p7 + p8 + p9) > 1 && (p2 + p3 + p4 + p5 + p6 + p7 + p8 + p9) < 7)
					{
						if (ap == 1)
						{
							if ((p2*p4*p6 == 0) && (p4*p6*p8 == 0))
							{
								dst.ptr<uchar>(i)[j] = 0;
								isFinished = true;
							}

							//   if((p2*p4*p8==0)&&(p2*p6*p8==0))
							//    {                           
							//         dst.ptr<uchar>(i)[j]=0;
							//         isFinished =TRUE;                            
							//    }

						}
					}
				}

			} //扫描过程一 结束


			dst.copyTo(tmpImg);
			//扫描过程二 开始
			for (i = 1; i < height; i++)  //一次 先行后列扫描 开始
			{
				pU = tmpImg.ptr<uchar>(i - 1);
				pC = tmpImg.ptr<uchar>(i);
				pD = tmpImg.ptr<uchar>(i + 1);
				for (int j = 1; j < width; j++)
				{
					if (pC[j] > 0)
					{
						int ap = 0;
						int p2 = (pU[j] > 0);
						int p3 = (pU[j + 1] > 0);
						if (p2 == 0 && p3 == 1)
						{
							ap++;
						}
						int p4 = (pC[j + 1] > 0);
						if (p3 == 0 && p4 == 1)
						{
							ap++;
						}
						int p5 = (pD[j + 1] > 0);
						if (p4 == 0 && p5 == 1)
						{
							ap++;
						}
						int p6 = (pD[j] > 0);
						if (p5 == 0 && p6 == 1)
						{
							ap++;
						}
						int p7 = (pD[j - 1] > 0);
						if (p6 == 0 && p7 == 1)
						{
							ap++;
						}
						int p8 = (pC[j - 1] > 0);
						if (p7 == 0 && p8 == 1)
						{
							ap++;
						}
						int p9 = (pU[j - 1] > 0);
						if (p8 == 0 && p9 == 1)
						{
							ap++;
						}
						if (p9 == 0 && p2 == 1)
						{
							ap++;
						}
						if ((p2 + p3 + p4 + p5 + p6 + p7 + p8 + p9) > 1 && (p2 + p3 + p4 + p5 + p6 + p7 + p8 + p9) < 7)
						{
							if (ap == 1)
							{
								//   if((p2*p4*p6==0)&&(p4*p6*p8==0))
								//   {                           
								//         dst.ptr<uchar>(i)[j]=0;
								//         isFinished =TRUE;                            
								//    }

								if ((p2*p4*p8 == 0) && (p2*p6*p8 == 0))
								{
									dst.ptr<uchar>(i)[j] = 0;
									isFinished = true;
								}

							}
						}
					}

				}

			} //一次 先行后列扫描完成          
			//如果在扫描过程中没有删除点，则提前退出
			if (isFinished == false)
			{
				break;
			}
		}

	}
}

void IPHelper::Extract_Points_Best_Upper(cv::Mat &src, vector<cv::Point> &Points, int threshold_value /* = 0 */)
{
	//Extract points which has largetest gray value
	//If every column has many max position, only extract upper one
	int i, j;
	//因为src.ptr<uchar>(j)是遍历行的，需要把图像转置，把列提取转为行提取
	cv::Mat src_t = src.t();
	int img_height = src_t.rows;
	int img_width = src_t.cols;
	for (j = 0; j < img_height; j++)
	{
		uchar* data = src_t.ptr<uchar>(j);
		double maxV = 0.0;
		int maxIndex = -1;
		for (i = 0; i < img_width; i++)
		{
			int grayValue = data[i];
			if (grayValue > maxV)//> operation confirms the upper one
			{
				maxV = grayValue;
				maxIndex = i;
			}
		}
		if (maxV >= threshold_value)
		{
			Points.push_back(cvPoint(j,maxIndex));//Check !!!
		}
		else
		{
			Points.push_back(cvPoint(j, 0));
		}
	}
	//End
}