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

// class to get data from Kinect v2
class MyKinectV2
{
public:
#pragma region donttouch
	CComPtr<IKinectSensor> kinect = nullptr;
	CComPtr<IColorFrameReader> colorFrameReader = nullptr;
	unsigned int colorBytesPerPixel;
	ColorImageFormat colorFormat = ColorImageFormat::ColorImageFormat_Bgra;
	CComPtr<IDepthFrameReader> depthFrameReader = nullptr;
	CComPtr<ICoordinateMapper> coordinateMapper;

	~MyKinectV2();
	MyKinectV2();

	int depthPointX, depthPointY;
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
#pragma endregion

#pragma region variables
	std::vector<BYTE> colorBuffer;
	int colorWidth, colorHeight;
	cv::Mat RGBImage;

	class HSVkeeper {
	public:
#pragma region variables
		cv::Scalar hsv_min;
		cv::Scalar hsv_max;
		cv::Mat hsvImage;
#pragma endregion

#pragma region donttouch
		int min_h, min_s, min_v;
		int max_h, max_s, max_v;
		cv::FileStorage fs;
		HSVkeeper() {
			fs.open("hsvValues.xml", cv::FileStorage::READ);
			fs["min_h"] >> min_h; fs["min_s"] >> min_s; fs["min_v"] >> min_v;
			fs["max_h"] >> max_h; fs["max_s"] >> max_s; fs["max_v"] >> max_v;
			fs.release();
		};
#pragma endregion
#pragma region functions
		void setHSVvalues() {
			hsv_min = cv::Scalar(min_h, min_s, min_v);
			hsv_max = cv::Scalar(max_h, max_s, max_v);
		};

		void setHSVImage(cv::Mat &img) { cv::cvtColor(img, hsvImage, CV_RGB2HSV); };
		void extractColor() {
			inRange(hsvImage, hsv_min, hsv_max, hsvImage);
			cv::medianBlur(hsvImage, hsvImage, 3);
		};
#pragma endregion
	};

	HSVkeeper hsvKeeper;

	std::vector<UINT16> depthBuffer;
	int depthWidth, depthHeight; //512 424
	cv::Mat depthImage;
	//深度の範囲
	UINT16 MINDEPTH = 3500, MAXDEPTH = 8000;
#pragma endregion

#pragma region functions
	//colorに関する関数
	void initializeColor();
	void updateColorFrame();
	void setRGB();
	void setMappedRGB();
	bool setRGBbyMovie(std::string movieName);

	virtual void initializeDepth();
	void updateDepthFrame();
	void setDepth();
	bool setDepthbyMovie(std::string movieName);
	void showDistance();
#pragma endregion


};