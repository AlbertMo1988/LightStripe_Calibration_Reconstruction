#pragma once
#include <string>
#include <vector>
#include "opencv2/opencv.hpp"
#pragma once

using namespace std;

class FileHelper
{
public:
	FileHelper();
	~FileHelper();

	void getFileList(const string& sPath, vector<string>& fileList, string file_suffix=".bmp");
	string getFileSuffix(const string& sFile);
	string getFileName(const string& sFile);
	void getFilePair_2(vector<string>& fileList, vector<vector<string>>& fileListPair, string key_suffix_1 = "tiff", string key_suffix_2 = "jpg");
	void sortFilelist(vector<string>& fileList);
	bool checkFileExist();

	void writeTxt(string& txtPath, vector<int>& data);
	void writeTxt(string& txtPath, int data);
	void readTxt(string& txtPath, vector<string>& data);

	int LoadData(string fileName, cv::Mat& matData, int matRows = 0, int matCols = 0, int matChns = 0);

private:
	string filePath;
};