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

	//color�Ɋւ���֐�
	void initializeColor();
	void updateColorFrame();
	void setRGB();

	//depth�Ɋւ���֐�
	virtual void initializeDepth();
	void updateDepthFrame();
	void setDepth();
	void binarization(cv::Mat image, const int minDepth, const int maxDepth);

	void getShuttleLoc();

	//line���o
	std::vector<cv::Vec4i> lines;
	virtual void getHoughLines(cv::Mat& src);

	//�|�[�����o
	char ringtype;

	vLine poleLine;

	//�[�x�͈̔͂��w��
	const UINT16 MINDEPTH = 3500, MAXDEPTH = 5000;

	//houghLines�̐��x
	const int RHO = 2, MINLINELENGTH = 180, MAXLINEGAP = 10, THRESHOLD = 50;
	const double ANGLETHRESHOLD = 0.90;

	//�V���g���R�b�N�̔����臒l
	const int FILTERTH = 300;  //�t�B���^�[:����ȉ��̒l��0�ɂ���
	const int SHATTLETH = 2000;  //�V���g���R�b�N�����݂��邩���肷��臒l
	const int KERNELSIZE = 10; //�J�[�l���̒���
	const float SUCCESSTH = 0.9;  //�����O�̔��a�ƃV���g���R�b�N�̋����̔��臒l

	//poleTop���X�V����t���[��
	const int updateFrame = 8;
	long long countFrame = 0;
	//�V���g���R�b�N���ʂ���������
	cv::Rect ringROI;
	cv::Mat ringImage;
	cv::Mat tempImage;
	virtual bool judgeCockThrough(); //ringtype=='g'�ŃS�[���f���V���g���R�b�N�p
	virtual bool findShuttle();
	cv::Point shuttleXY, shuttleXY_suc;

};