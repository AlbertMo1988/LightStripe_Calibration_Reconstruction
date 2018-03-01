#include "FileHelper.h"
#include <io.h>
#include <stdlib.h>
#include <fstream>
#include <iostream>
#include <string>
#include <sstream>
#include <iterator>
#include <Windows.h>

using namespace std;

FileHelper::FileHelper()
{}
FileHelper::~FileHelper()
{}

int str2int(const string& inputStr)
{
	int number;
	stringstream ss;
	ss << inputStr;
	ss >> number;
	return number;
}

string int2str(const int inputInt)
{
	stringstream ss;
	ss << inputInt;
	string tempStr = ss.str();
	return tempStr;
}

void getFile(const string& sPath, vector<string>& fileList, _finddata_t& file,string file_suffix)
{
	string sAddPath = sPath;
	sAddPath += "\\";
	sAddPath += file.name;
	fileList.push_back(sAddPath);
}
void FileHelper::getFileList(const string& sPath, vector<string>& fileList, string file_suffix)
{
	string sPathLast = sPath + "\\"+"*"+file_suffix;//*.*-->ALL
	struct _finddata_t file;
	intptr_t hFile;
	hFile = _findfirst(sPathLast.c_str(), &file);
	if (hFile == -1)
	{
		return;
	}
	else
	{
		if (file.size!=0)
		{
			getFile(sPath, fileList, file, file_suffix);
		}
	}

	while (_findnext(hFile, &file) != -1)
	{
		if (file.size != 0)
		{
			getFile(sPath, fileList, file, file_suffix);
		}
	}
}
bool FileHelper::checkFileExist()
{
	fstream _file;
	_file.open(filePath, ios::in);
	if (!_file)
	{
		return false;
	}
	else
	{
		return true;
	}
}
string FileHelper::getFileSuffix(const string& sFile)
{
	if (sFile.rfind(".") != string::npos)
	{
		return sFile.substr(sFile.rfind(".") + 1);
	}
	else
	{
		return "none";
	}
}
string FileHelper::getFileName(const string& sFile)
{
	if (sFile.rfind(".") != string::npos && sFile.rfind("\\") != string::npos)
	{
		string temp = sFile.substr(sFile.rfind("\\") + 1);
		return temp.substr(0, temp.rfind("."));
	}
}
void FileHelper::getFilePair_2(vector<string>& fileList, vector<vector<string>>& fileListPair, string key_suffix_1, string key_suffix_2)
{
	int n = fileList.size();
	vector<string> fileNameList(fileList);
	for (size_t i = 0; i < n; i++)
	{
		string t = fileNameList[i];
		t = getFileName(t);
		fileNameList[i] = t;
	}
	for (int i = 0; i < n; i++)
	{
		vector<string> tempPair;
		string key_file_1 = fileNameList[i];
		for (int j = i+1; j < n; j++)
		{
			string key_file_2 = fileNameList[j];
			if (key_file_1==key_file_2)
			{
				tempPair.push_back(key_file_1 + "." + key_suffix_1);
				tempPair.push_back(key_file_2 + "." + key_suffix_2);
				fileListPair.push_back(tempPair);
				fileNameList.erase(fileNameList.begin()+j);
				fileNameList.erase(fileNameList.begin()+i);
				n = n - 2;
				i--;
			}
		}
	}
}
void FileHelper::sortFilelist(vector<string>& fileList)
{
	unsigned int listSize = fileList.size();
	if (listSize<1)
	{
		return;
	}
	string tempStr = fileList[0];
	string filePath = tempStr.substr(0, tempStr.rfind("\\"));
	string fileSuffix = tempStr.substr(tempStr.rfind(".") + 1);
	vector<int> fileNameList;
	for (unsigned int i = 0; i < listSize; i++)
	{
		string tempFileFullName = fileList[i];
		tempFileFullName = getFileName(tempFileFullName);
		fileNameList.push_back(str2int(tempFileFullName));
	}
	sort(fileNameList.begin(), fileNameList.end());
	for (unsigned int i = 0; i < listSize; i++)
	{
		fileList[i] = filePath + "\\" + int2str(fileNameList[i]) + "." + fileSuffix;
	}
}

void FileHelper::writeTxt(string& txtPath, vector<int>& data)
{
	ofstream outFile(txtPath);
	if (!outFile)
	{
		cout << "Unable to open outfile";
		return;
	}
	int i = 0;
	int dataLength = data.size();
	for (i = 0; i < dataLength;i++)
	{
		outFile << data[i] << endl;
	}
	outFile.close();
	return;
}
void FileHelper::writeTxt(string& txtPath, int data)
{
	ofstream outFile(txtPath);
	if (!outFile)
	{
		cout << "Unable to open outfile";
		return;
	}
	outFile << data << endl;
	outFile.close();
	return;
}

void FileHelper::readTxt(string& txtPath, vector<string>& data)
{
	char buffer[256];
	ifstream inputFile(txtPath);
	if (!inputFile)
	{
		cout << "Unable to open outfile";
		return;
	}
	//int a, b;
	int i = 0;
	while (!inputFile.eof())
	{
		inputFile.getline(buffer, 10);
		data.push_back(buffer);
		//sscanf(buffer, "%d %d", &a, &b);
		//cout << a << " " << b << endl;
		//data[i][0] = a;
		//data[i][1] = b;
		i++;
	}
}

int FileHelper::LoadData(string fileName, cv::Mat& matData, int matRows, int matCols, int matChns)
{
	int retVal = 0;

	// ���ļ�  
	ifstream inFile(fileName.c_str(), ios_base::in);
	if (!inFile.is_open())
	{
		cout << "��ȡ�ļ�ʧ��" << endl;
		retVal = -1;
		return (retVal);
	}

	// ��������  
	istream_iterator<double> begin(inFile);    //�� float ��ʽȡ�ļ�����������ʼָ��  
	istream_iterator<double> end;          //ȡ�ļ�������ֹλ��  
	vector<double> inData(begin, end);      //���ļ����ݱ����� std::vector ��  
	cv::Mat tmpMat = cv::Mat(inData);       //�������� std::vector ת��Ϊ cv::Mat  

	// ����������д���  
	//copy(vec.begin(),vec.end(),ostream_iterator<double>(cout,"\t"));   

	// ����趨�ľ���ߴ��ͨ����  
	size_t dataLength = inData.size();
	//1.ͨ����  
	if (matChns == 0)
	{
		matChns = 1;
	}
	//2.������  
	if (matRows != 0 && matCols == 0)
	{
		matCols = dataLength / matChns / matRows;
	}
	else if (matCols != 0 && matRows == 0)
	{
		matRows = dataLength / matChns / matCols;
	}
	else if (matCols == 0 && matRows == 0)
	{
		matRows = dataLength / matChns;
		matCols = 1;
	}
	//3.�����ܳ���  
	if (dataLength != (matRows * matCols * matChns))
	{
		cout << "��������ݳ��� ������ �趨�ľ���ߴ���ͨ����Ҫ�󣬽���Ĭ�Ϸ�ʽ�������" << endl;
		retVal = 1;
		matChns = 1;
		matRows = dataLength;
	}

	// ���ļ����ݱ������������  
	matData = tmpMat.reshape(matChns, matRows).clone();

	return (retVal);
}
