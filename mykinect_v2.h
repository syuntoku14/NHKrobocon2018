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
protected:

	CComPtr<IKinectSensor> kinect = nullptr;
	CComPtr<IColorFrameReader> colorFrameReader = nullptr;
	unsigned int colorBytesPerPixel;
	ColorImageFormat colorFormat = ColorImageFormat::ColorImageFormat_Bgra;
	CComPtr<IDepthFrameReader> depthFrameReader = nullptr;

public:
	~MyKinectV2();
	MyKinectV2();

	std::vector<BYTE> colorBuffer;
	int colorWidth, colorHeight;
	cv::Mat RGBImage;

	std::vector<UINT16> depthBuffer;
	int depthWidth, depthHeight; //523 424
	int depthPointX, depthPointY;
	const char* DepthWindowName = "Depth Image";
	cv::Mat depthImage;


	//poleLine���X�V����t���[��
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

	//�|�[�����o
	Pole poleLine;

	//color�Ɋւ���֐�
	void initializeColor();
	void updateColorFrame();
	void setRGB();

	class HSVkeeper {
	public:
		int min_h, min_s, min_v;
		int max_h, max_s, max_v;
		cv::Scalar hsv_min;
		cv::Scalar hsv_max;
		void setHSVvalues() {
			hsv_min = cv::Scalar(min_h, min_s, min_v);
			hsv_max = cv::Scalar(max_h, max_s, max_v);
		};
		cv::Mat hsvImage;
		void setHSVImage(cv::Mat &img) { cv::cvtColor(img, hsvImage, CV_RGB2HSV); };
		void extractColor() { inRange(hsvImage, hsv_min, hsv_max, hsvImage); };
		void showHSVImage() { 
			cv::Mat img;
			cv::resize(hsvImage, img, cv::Size(), 0.3, 0.3);
			cv::imshow("HSVImage", img);
		};
	};

	HSVkeeper hsvKeeper;


	//depth�Ɋւ���֐�
	virtual void initializeDepth();
	void updateDepthFrame();
	void setDepth();

	//�[�x�͈̔�
	UINT16 MINDEPTH = 3500, MAXDEPTH = 8000;
	void binarization(cv::Mat &image, const int minDepth, const int maxDepth);

	void getShuttleLoc();

	virtual void getHoughLines(cv::Mat& src);

	//houghLines�̐��x
	const int RHO = 2, MINLINELENGTH = 180, MAXLINEGAP = 10, THRESHOLD = 50;
	const double ANGLETHRESHOLD = 0.90;

	//�V���g���R�b�N�̔����臒l
	const int FILTERTH =150;  //�t�B���^�[:����ȉ��̒l��0�ɂ���
	const int SHATTLETH = 2000;  //�V���g���R�b�N�����݂��邩���肷��臒l
	const int KERNELSIZE = 10; //�J�[�l���̒���
	const float SUCCESSTH = 0.9;  //�����O�̔��a�ƃV���g���R�b�N�̋����̔��臒l


	//�V���g���R�b�N���ʂ���������
	virtual bool judgeCockThrough(); //ringtype=='g'�ŃS�[���f���V���g���R�b�N�p
	virtual bool findShuttle();

};