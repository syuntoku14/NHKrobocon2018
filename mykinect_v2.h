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

class MyKinectV2
{
public:
#pragma region CComPtr
	CComPtr<IKinectSensor> kinect = nullptr;
	CComPtr<IColorFrameReader> colorFrameReader = nullptr;
	unsigned int colorBytesPerPixel;
	ColorImageFormat colorFormat = ColorImageFormat::ColorImageFormat_Bgra;
	CComPtr<IDepthFrameReader> depthFrameReader = nullptr;
	CComPtr<ICoordinateMapper> coordinateMapper;
#pragma endregion

	~MyKinectV2();
	MyKinectV2();

#pragma region color

	std::vector<BYTE> colorBuffer;
	int colorWidth, colorHeight;
	cv::Mat RGBImage;

	//colorに関する関数
	void initializeColor();
	void updateColorFrame();
	void setRGB();
	void setMappedRGB();

	class HSVkeeper {
	public:
		int min_h, min_s, min_v;
		int max_h, max_s, max_v;
		cv::Scalar hsv_min;
		cv::Scalar hsv_max;

		cv::FileStorage fs;
		HSVkeeper() {
			fs.open("hsvValues.xml", cv::FileStorage::READ);
			fs["min_h"] >> min_h; fs["min_s"] >> min_s; fs["min_v"] >> min_v;
			fs["max_h"] >> max_h; fs["max_s"] >> max_s; fs["max_v"] >> max_v;
			fs.release();
		};

		void setHSVvalues() {
			hsv_min = cv::Scalar(min_h, min_s, min_v);
			hsv_max = cv::Scalar(max_h, max_s, max_v);
		};
		cv::Mat hsvImage;
		void setHSVImage(cv::Mat &img) { cv::cvtColor(img, hsvImage, CV_RGB2HSV); };
		void extractColor() {
			inRange(hsvImage, hsv_min, hsv_max, hsvImage);
			cv::medianBlur(hsvImage, hsvImage, 3);
		};
		void showHSV() {
			cv::Mat img;
			cv::resize(hsvImage, img, cv::Size(), 0.3, 0.3);
			cv::imshow("HSVImage", img);
		};
	};

	HSVkeeper hsvKeeper;
#pragma endregion

#pragma region depth
	std::vector<UINT16> depthBuffer;
	int depthWidth, depthHeight; //523 424
	int depthPointX, depthPointY;
	const char* DepthWindowName = "Depth Image";
	cv::Mat depthImage;

	virtual void initializeDepth();
	void updateDepthFrame();
	void setDepth();
	//深度の範囲
	UINT16 MINDEPTH = 3500, MAXDEPTH = 8000;
#pragma endregion

#pragma region pole
	const int updateFrame = 8;
	long long countFrame = 0;
	class Pole {
	public:
		char ringtype;
		int length;
		cv::Vec4i line;
		std::vector<int> top{ -1,-1 };
		cv::Rect ringROI;
		cv::Mat ringImage;
		cv::Point shuttleXY, shuttleXY_suc;

		Pole(int length, cv::Vec4i line) {
			this->length = length;
			this->line = line;
		}
		Pole() { this->length = -1; };
	};

	Pole poleLine;

	virtual void getHoughLines(cv::Mat& src);

	//houghLinesの精度
	const int RHO = 2, MINLINELENGTH = 180, MAXLINEGAP = 10, THRESHOLD = 50;
	const double ANGLETHRESHOLD = 0.90;
#pragma endregion

	//シャトルコックの判定の閾値
	const int FILTERTH =150;  //フィルター:これ以下の値は0にする
	const int SHATTLETH = 2000;  //シャトルコックが存在するか判定する閾値
	const int KERNELSIZE = 10; //カーネルの長さ
	const float SUCCESSTH = 0.9;  //リングの半径とシャトルコックの距離の比の閾値

	void binarization(cv::Mat &image, const int minDepth, const int maxDepth);
	void getShuttleLoc();
	virtual bool judgeCockThrough(); //ringtype=='g'でゴールデンシャトルコック用
	virtual bool findShuttle();

};