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
	//colordata�p
	CComPtr<IColorFrameReader> colorFrameReader = nullptr;
	unsigned int colorBytesPerPixel;
	ColorImageFormat colorFormat = ColorImageFormat::ColorImageFormat_Bgra;
	std::vector<BYTE> colorBuffer;
	int colorWidth;
	int colorHeight;
	
	//depthdata�p
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

	//color�Ɋւ���֐�
	void initializeColor(); 	
	void runColor();
	//depth�Ɋւ���֐�
	void initializeDepth();
	void runDepth();
	void showDistance();

	static void mouseCallback(int event, int x, int y, int flags, void* userdata) {
		// �����ɓn����this�|�C���^���o�R���ă����o�֐��ɓn��
		auto pThis = (MyKinectV2*)userdata;
		pThis->mouseCallback(event, x, y, flags);
	};
	void mouseCallback(int event, int x, int y, int flags) {
		if (event == CV_EVENT_LBUTTONDOWN) {
			depthPointX = x;
			depthPointY = y;
		}
	};

	//color�f�[�^�̏���
	void updateColor(); 
	void updateColorFrame();
	void drawColor();
	void drawColorFrame(); 

	//depth�f�[�^�̏���
	void updateDepth(); 
	void updateDepthFrame();
	void drawDepth();
	void drawDepthFrame();

	//line���o
	std::vector<cv::Vec4i> lines;
	void getHoughLines(cv::Mat& src);

	//�|�[�����o
	char ringtype;
	int poleX; //0~depthWidth�̂ǂꂩ MINLENGTH��菬�����l�̎���-1
	int poleLength;//�|�[���̒��� pixel
	std::vector<int> poleTop{-1,-1}; //pole�̒��_�̍��W �|�[�����Ȃ��Ƃ���(x,y)��-1,-1������
	float sumTopX;
	float sumTopY;
	bool sucFlag=false;

	//�V���g���R�b�N���ʂ���������
	cv::Rect ringROI;
	cv::Mat ringImage;
	cv::Mat tempImage;
	bool judgeCockThrough(); //ringtype=='g'�ŃS�[���f���V���g���R�b�N�p
	cv::Scalar ringSumB4;
	cv::Scalar ringSum;
	bool findShuttle();
	cv::Mat filteredRing; //ringImage�Ɠ����T�C�Y,�^
	cv::Point shuttleXYB4;
	cv::Point shuttleXY;
	cv::Point shuttleXY_suc;
	//void findPoleX();
	//circle���o
	//std::vector<cv::Vec3f> circles;
	//void getHoughCircles(cv::Mat &src);

	void saveDepthMovie();
	void useDepthMovie();
};