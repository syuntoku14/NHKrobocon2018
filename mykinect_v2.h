#pragma once
#include<Kinect.h>
#include<opencv2\opencv.hpp>
#include<atlbase.h>
#include <iostream>
#include <sstream>
#include <map>

#define ERROR_CHECK( ret )  \
    if ( (ret) != S_OK ) {    \
        std::stringstream ss;	\
        ss << "failed " #ret " " << std::hex << ret << std::endl;			\
        throw std::runtime_error( ss.str().c_str() );			\
    } \

class MyKinectV2
{
private:
	// Kinect SDK
	CComPtr<IKinectSensor> kinect = nullptr;
	//colordata用
	CComPtr<IColorFrameReader> colorFrameReader = nullptr;
	unsigned int colorBytesPerPixel;
	ColorImageFormat colorFormat = ColorImageFormat::ColorImageFormat_Bgra;
	std::vector<BYTE> colorBuffer;
	int colorWidth;
	int colorHeight;
	
	//depthdata用
	CComPtr<IDepthFrameReader> depthFrameReader = nullptr;
	std::vector<UINT16> depthBuffer;
	int depthWidth; //512
	int depthHeight; //424
	int depthPointX;
	int depthPointY;
	const char* DepthWindowName = "Depth Image";
	cv::Mat depthImage;

public:
	~MyKinectV2();
	MyKinectV2();

	//colorに関する関数
	void initializeColor(); 	
	void runColor();
	//depthに関する関数
	void initializeDepth();
	void runDepth();
	void showDistance();

	static void mouseCallback(int event, int x, int y, int flags, void* userdata) {
		// 引数に渡したthisポインタを経由してメンバ関数に渡す
		auto pThis = (MyKinectV2*)userdata;
		pThis->mouseCallback(event, x, y, flags);
	};
	void mouseCallback(int event, int x, int y, int flags) {
		if (event == CV_EVENT_LBUTTONDOWN) {
			depthPointX = x;
			depthPointY = y;
		}
	};

	//colorデータの処理
	void updateColor(); 
	void updateColorFrame();
	void drawColor();
	void drawColorFrame(); 

	//depthデータの処理
	void updateDepth(); 
	void updateDepthFrame();
	void drawDepth();
	void drawDepthFrame();

	//line検出
	std::vector<cv::Vec4i> lines;
	void getHoughLines(cv::Mat& src);

	//ポール検出
	char ringtype;
	int poleX; //0~depthWidthのどれか MINLENGTHより小さい値の時は-1
	int poleLength;//ポールの長さ pixel
	std::vector<int> poleTop{-1,-1}; //poleの頂点の座標 ポールがないときは(x,y)に-1,-1が入る
	float sumTopX;
	float sumTopY;
	bool sucFlag=false;

	//シャトルコックが通ったか判定
	cv::Rect ringROI;
	cv::Mat ringImage;
	cv::Mat tempImage;
	bool judgeCockThrough(); //ringtype=='g'でゴールデンシャトルコック用
	cv::Scalar ringSumB4;
	cv::Scalar ringSum;
	bool findShuttle();
	cv::Mat filteredRing; //ringImageと同じサイズ,型
	cv::Point shuttleXYB4;
	cv::Point shuttleXY;
	cv::Point shuttleXY_suc;
	//void findPoleX();
	//circle検出
	//std::vector<cv::Vec3f> circles;
	//void getHoughCircles(cv::Mat &src);

	void saveDepthMovie();
	void useDepthMovie();
};