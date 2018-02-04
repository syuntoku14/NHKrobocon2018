#pragma once
#include<Kinect.h>
#include<opencv2\opencv.hpp>
#include<atlbase.h>
#include <iostream>
#include <sstream>
#include <map>
#include<queue>

#define ERROR_CHECK( ret )  \
    if ( (ret) != S_OK ) {    \
        std::stringstream ss;	\
        ss << "failed " #ret " " << std::hex << ret << std::endl;			\
        throw std::runtime_error( ss.str().c_str() );			\
    } \

class vLine {
public:
	int length;
	cv::Vec4i line;
	std::vector<int> top{ -1,-1 };
	vLine(int length, cv::Vec4i line) {
		this->length = length;
		this->line = line;
	}
	vLine() { this->length = -1; };
};

class MyKinectV2
{
protected:
	CComPtr<IKinectSensor> kinect = nullptr;
	CComPtr<IColorFrameReader> colorFrameReader = nullptr;
	unsigned int colorBytesPerPixel;
	ColorImageFormat colorFormat = ColorImageFormat::ColorImageFormat_Bgra;
	CComPtr<IDepthFrameReader> depthFrameReader = nullptr;

	std::vector<BYTE> colorBuffer;
	int colorWidth, colorHeight;
	cv::Mat RGBImage;

	std::vector<UINT16> depthBuffer;
	int depthWidth, depthHeight; //523 424
	int depthPointX, depthPointY;
	const char* DepthWindowName = "Depth Image";
	cv::Mat depthImage;

public:
	~MyKinectV2();
	MyKinectV2();

	//colorに関する関数
	void initializeColor();
	void updateColorFrame();
	void setRGB();

	//depthに関する関数
	virtual void initializeDepth();
	void updateDepthFrame();
	void setDepth();
	void binarization(cv::Mat image, const int minDepth, const int maxDepth);

	void getShuttleLoc();

	//line検出
	std::vector<cv::Vec4i> lines;
	virtual void getHoughLines(cv::Mat& src);

	//ポール検出
	char ringtype;

	vLine poleLine;

	//深度の範囲を指定
	const UINT16 MINDEPTH = 3500, MAXDEPTH = 5000;

	//houghLinesの精度
	const int RHO = 2, MINLINELENGTH = 180, MAXLINEGAP = 10, THRESHOLD = 50;
	const double ANGLETHRESHOLD = 0.90;

	//シャトルコックの判定の閾値
	const int FILTERTH = 300;  //フィルター:これ以下の値は0にする
	const int SHATTLETH = 2000;  //シャトルコックが存在するか判定する閾値
	const int KERNELSIZE = 10; //カーネルの長さ
	const float SUCCESSTH = 0.9;  //リングの半径とシャトルコックの距離の比の閾値

	//poleTopを更新するフレーム
	const int updateFrame = 8;
	long long countFrame = 0;
	//シャトルコックが通ったか判定
	cv::Rect ringROI;
	cv::Mat ringImage;
	cv::Mat tempImage;
	virtual bool judgeCockThrough(); //ringtype=='g'でゴールデンシャトルコック用
	virtual bool findShuttle();
	cv::Point shuttleXY, shuttleXY_suc;

};