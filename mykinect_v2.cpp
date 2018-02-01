#include"mykinect_v2.h"
namespace{	
//�[�x�͈̔͂��w��
const UINT16 DEPTHRANGEMIN = 3500;
const UINT16 DEPTHRANGEMAX = 5000;
 
//����̕ۑ���Ɨ��p��
std::time_t now = std::time(nullptr);
std::string writeName = "./shuttleMovies/"+std::to_string(now)+".avi";
std::string useName = "./shuttleMovies/mis1.avi";

//houghLines�̐��x
const int RHO=2; //�s�N�Z���P�ʂ̓��[��Ԃ̋�������\
const int MINLINELENGTH=180; //����ȉ��̒����͊��p
const int MAXLINEGAP=10;  //2�_����������ɂ���Ƃ݂Ȃ��ő勗��
const int THRESHOLD=50;   //���[��臒l�p�����[�^
 
const double ANGLETHRESHOLD = 0.90;

//houghCircles�̐��x
const int DP=2; //���[��Ԃ̕���\�̔䗦
const int MINDIST=200;  //�~�̒��S���m�̋���
const int PARAM1=50;
const int PARAM2=90;	//���[����臒l
const int MINRADIUS=0;  //�~�̔��a�̍ŏ��l
const int MAXRADIUS=0;  //�~�̔��a�̍ő�l

//�V���g���R�b�N�̔����臒l
const int FILTERTH = 150;  //�t�B���^�[���|�������ɂ���ȉ��̒l��0�ɂ���
const int SHATTLETH=2000;  //�V���g���R�b�N�����݂��邩���肷��臒l
const int KERNELLENGTH = 10; //�J�[�l���̕ӂ̒���
double kernel[::KERNELLENGTH][::KERNELLENGTH];
const float SUCCESSTH = 0.9;  //�����O�̔��a�ƃV���g���R�b�N�̋����̔��臒l

std::vector<int> poleLine(4,-1);

//poleTop���X�V����t���[��
const int updateFrame = 8;
long long countFrame=0;
}

MyKinectV2::MyKinectV2() {
	// �f�t�H���g��Kinect���擾����
	ERROR_CHECK(::GetDefaultKinectSensor(&kinect));
	ERROR_CHECK(kinect->Open());
	ringSum[0] = -1e8;
	ringtype = 'g';
	for (int i = 0; i < ::KERNELLENGTH; i++) {
		for (int j = 0; j < ::KERNELLENGTH; j++) {
			kernel[i][j] = 1.0/pow(KERNELLENGTH,2);
		}
	}
}

MyKinectV2::~MyKinectV2()
{
	// Kinect�̓�����I������
	if (kinect != nullptr) {
		kinect->Close();
	}
}

void MyKinectV2::initializeColor() {
	// �J���[���[�_�[���擾����
	CComPtr<IColorFrameSource> colorFrameSource;
	ERROR_CHECK(kinect->get_ColorFrameSource(&colorFrameSource));
	ERROR_CHECK(colorFrameSource->OpenReader(&colorFrameReader));

	// �f�t�H���g�̃J���[�摜�̃T�C�Y���擾����
	CComPtr<IFrameDescription> defaultColorFrameDescription;
	ERROR_CHECK(colorFrameSource->get_FrameDescription(&defaultColorFrameDescription));
	ERROR_CHECK(defaultColorFrameDescription->get_Width(&colorWidth));
	ERROR_CHECK(defaultColorFrameDescription->get_Height(&colorHeight));
	ERROR_CHECK(defaultColorFrameDescription->get_BytesPerPixel(&colorBytesPerPixel));
	std::cout << "default : " << colorWidth << ", " << colorHeight << ", " << colorBytesPerPixel << std::endl;

	// �J���[�摜�̃T�C�Y���擾����
	CComPtr<IFrameDescription> colorFrameDescription;
	ERROR_CHECK(colorFrameSource->CreateFrameDescription(
		colorFormat, &colorFrameDescription));
	ERROR_CHECK(colorFrameDescription->get_Width(&colorWidth));
	ERROR_CHECK(colorFrameDescription->get_Height(&colorHeight));
	ERROR_CHECK(colorFrameDescription->get_BytesPerPixel(&colorBytesPerPixel));
	std::cout << "create  : " << colorWidth << ", " << colorHeight << ", " << colorBytesPerPixel << std::endl;

	// �o�b�t�@�[���쐬����
	colorBuffer.resize(colorWidth * colorHeight * colorBytesPerPixel);
}

void MyKinectV2::runColor() {
	while (1) {
		updateColor();
		drawColor();

		auto key = cv::waitKey(10);
		if (key == 'q') {
			break;
		}
	}
}

// �f�[�^�̍X�V����
void MyKinectV2::updateColor() {
	updateColorFrame();
}

// �J���[�t���[���̍X�V
void MyKinectV2::updateColorFrame() {
	// �t���[�����擾����
	CComPtr<IColorFrame> colorFrame;
	auto ret = colorFrameReader->AcquireLatestFrame(&colorFrame);
	if (FAILED(ret)) {
		return;
	}

	// �w��̌`���Ńf�[�^���擾����
	ERROR_CHECK(colorFrame->CopyConvertedFrameDataToArray(
		colorBuffer.size(), &colorBuffer[0], colorFormat));
}

void MyKinectV2::drawColor() {
	drawColorFrame();
}

//�����ɏ���������
void MyKinectV2::drawColorFrame() {
#if 0
	// �J���[�f�[�^��\������
	cv::Mat colorImage(colorHeight, colorWidth, CV_8UC4, &colorBuffer[0]);
	cv::imshow("Color Image", colorImage);
#else
	cv::Mat colorImage(colorHeight, colorWidth, CV_8UC4, &colorBuffer[0]);
	cv::Mat harfImage;
	cv::resize(colorImage, harfImage, cv::Size(), 0.5, 0.5);
	cv::imshow("Harf Image", harfImage);
#endif
}

void MyKinectV2::initializeDepth() {
	// Depth���[�_�[���擾����
	CComPtr<IDepthFrameSource> depthFrameSource;
	ERROR_CHECK(kinect->get_DepthFrameSource(&depthFrameSource));
	ERROR_CHECK(depthFrameSource->OpenReader(&depthFrameReader));

	// Depth�摜�̃T�C�Y���擾����
	CComPtr<IFrameDescription> depthFrameDescription;
	ERROR_CHECK(depthFrameSource->get_FrameDescription(
		&depthFrameDescription));
	ERROR_CHECK(depthFrameDescription->get_Width(&depthWidth));
	ERROR_CHECK(depthFrameDescription->get_Height(&depthHeight));

	//�}�E�X�̏����ʒu(�ʂɂȂ�ł��ǂ�)
	depthPointX = depthWidth / 2;
	depthPointY = depthHeight / 2;

	// Depth�̍ő�l�A�ŏ��l���擾����
	UINT16 minDepthReliableDistance;
	UINT16 maxDepthReliableDistance;
	ERROR_CHECK(depthFrameSource->get_DepthMinReliableDistance(
		&minDepthReliableDistance));
	ERROR_CHECK(depthFrameSource->get_DepthMaxReliableDistance(
		&maxDepthReliableDistance));
	std::cout << "Depth�ŏ��l       : " << minDepthReliableDistance << std::endl;
	std::cout << "Depth�ő�l       : " << maxDepthReliableDistance << std::endl;
	std::cout << "depthWidth: " << depthWidth << std::endl;
	std::cout << "depthHeight: " << depthHeight << std::endl;

	// �o�b�t�@�[���쐬����
	depthBuffer.resize(depthWidth * depthHeight);

	// �}�E�X�N���b�N�̃C�x���g��o�^����
	cv::namedWindow(DepthWindowName);
	cv::setMouseCallback(DepthWindowName, &MyKinectV2::mouseCallback, this);

}

void MyKinectV2::runDepth() {
	while (1) {
		updateDepth();
		drawDepth();

		auto key = cv::waitKey(10);
		if (key == 'q') {
			break;
		}
	}
}

void MyKinectV2::updateDepth() {
	updateDepthFrame();
}

void MyKinectV2::updateDepthFrame() {
	// Depth�t���[�����擾����
	CComPtr<IDepthFrame> depthFrame;
	auto ret = depthFrameReader->AcquireLatestFrame(&depthFrame);
	if (ret != S_OK) {
		return;
	}

	// �f�[�^���擾����
	ERROR_CHECK(depthFrame->CopyFrameDataToArray(
		depthBuffer.size(), &depthBuffer[0]));
}

void MyKinectV2::drawDepth() {
	drawDepthFrame();
}

//�f���̕ۑ�
void MyKinectV2::saveDepthMovie() {
	static cv::VideoWriter writer(writeName, cv::VideoWriter::fourcc('X', 'V', 'I', 'D'), 30, cv::Size(depthWidth, depthHeight), false);
	// Depth�f�[�^��\������
	depthImage = cv::Mat(depthHeight, depthWidth, CV_8UC1);

	//�w�肵�������̂�2�l������depthImage�Ɋi�[
	for (int i = 0; i < depthImage.total(); ++i) {
		if (DEPTHRANGEMIN < depthBuffer[i] && depthBuffer[i] < DEPTHRANGEMAX) {
			//depthImage.data[i] = depthBuffer[i] % 255;
			//depthImage.data[i] = (unsigned char)(((float)depthBuffer[i] / 8000.0)* 255.0);
			depthImage.data[i] = 255;  //2�l��
		}
		else {
			depthImage.data[i] = 0;
		}
	}
	writer << depthImage;
	imshow("depthImage", depthImage);
}

//�f�����g���ď����i�f�o�b�O�p�j
void MyKinectV2::useDepthMovie() {
	depthImage = cv::Mat(depthHeight, depthWidth, CV_8UC1);
	static cv::VideoCapture cap(useName);
	cv::Mat temp;

	while (1) {
		cap >> temp;
		if (temp.empty()) {
			break;
		}
		auto key = cv::waitKey(1);
		if (key == 'q') {
			break;
		}
		else if (key == 's') {
			cv::waitKey(0);
		}
		cv::cvtColor(temp, depthImage, CV_RGB2GRAY);

		//������
		cv::medianBlur(depthImage, depthImage, 3);
		showDistance();
		getHoughLines(depthImage);
		if (judgeCockThrough()) {
			sucFlag=findShuttle();
			if (sucFlag) {
				std::cout << "success!" << std::endl;
			}
		}
		if (countFrame > 530) {
			std::cout << "Fault" << std::endl;
		}
	}
}

//����ꂽ������depthBuffer�Ɋi�[�����
//�����ɏ���������
void MyKinectV2::drawDepthFrame() {
	// Depth�f�[�^��\������
	depthImage=cv::Mat(depthHeight, depthWidth, CV_8UC1);
		//�w�肵�������̂�2�l������depthImage�Ɋi�[
	for (int i = 0; i < depthImage.total(); ++i) {
		if (DEPTHRANGEMIN < depthBuffer[i] && depthBuffer[i] < DEPTHRANGEMAX) {
			//depthImage.data[i] = depthBuffer[i] % 255;
			//depthImage.data[i] = (unsigned char)(((float)depthBuffer[i] / 8000.0)* 255.0);
			depthImage.data[i] = 255;  //2�l��
		}
		else {
			depthImage.data[i] = 0;
		}
	}

	//������
	cv::medianBlur(depthImage, depthImage, 3);

	showDistance();
	getHoughLines(depthImage);
	if (judgeCockThrough()) {
		if (findShuttle()) {
			std::cout << "success!" << std::endl;
		}
	}
}

void MyKinectV2::showDistance() {
	// Depth�f�[�^�̃C���f�b�N�X���擾���āA���̏ꏊ�̋�����\������
	int index = (depthPointY * depthWidth) + depthPointX;
	std::stringstream ss;
	ss << depthBuffer[index] << "mm";

	cv::circle(depthImage, cv::Point(depthPointX, depthPointY), 10,
		cv::Scalar(0, 0, 255), 2);
	cv::putText(depthImage, ss.str(), cv::Point(depthPointX, depthPointY),
		0, 1, cv::Scalar(255, 255, 255));
	cv::imshow(DepthWindowName, depthImage);
}

void MyKinectV2::getHoughLines(cv::Mat& src) {
	using namespace std;
	using namespace cv;

	static Mat dst, color_dst;
	static string str_x;

	//canny�ϊ�
	Canny(src, dst, 50, 200, 3);
	cvtColor(dst, color_dst, CV_GRAY2BGR);
	HoughLinesP(dst, lines, ::RHO, CV_PI / 360.0, ::THRESHOLD, ::MINLINELENGTH, ::MAXLINEGAP);

	std::vector<cv::Vec4i> verticalLines;
	std::vector<double> verticalLength;

	//���ȏ�̊p�x�̂��̂𒊏o
	for (size_t i = 0; i < lines.size(); i++) {
		double angle = atan(abs(lines[i][1] - lines[i][3]) / (abs(lines[i][0] - lines[i][2]) + 1e-10));
		if (lines[i][0] < 507 && lines[i][0]>5) { //�[�������o�O�̌����ɂȂ邩��
			if (angle > CV_PI / 2.0*::ANGLETHRESHOLD) {
				verticalLines.push_back(lines[i]);
				verticalLength.push_back((int)sqrt(pow((lines[i][0] - lines[i][2]), 2) + pow((lines[i][1] - lines[i][3]), 2)));
			}
		}
	}

	int longestItr = 0;
	double longestLength = 0.0;
	for (size_t i = 0; i < verticalLines.size(); i++)
	{
		//��Ԓ�������������
		if (verticalLength[longestItr] <= verticalLength[i]) {
			longestLength = verticalLength[i];
			longestItr = i;
		}
	}

	//line���������Ă���ꍇ�̏���
	if (verticalLines.size() > 0) {
		//��Ԓ�������poleLine�ɓ���� updateFrame�t���[������
		if (countFrame % updateFrame == 0) {
			for (int i = 0; i < 4; i++) {
				poleLine[i] = verticalLines[longestItr][i];
			}
			poleLength = (int)longestLength;
		}
	}
	
	if(poleLine[0]>0){
		//poleLine��`��i�f�o�b�O�p�j
		line(color_dst, Point(poleLine[0], poleLine[1]), Point(poleLine[2], poleLine[3]), Scalar(0, 0, 255), 3, 8);
		poleX = poleLine[0];
		//pole�̒��_�̍��W���i�[
		if (poleLine[1] >= poleLine[3]) {
			sumTopX += poleLine[2]; sumTopY += poleLine[3];
		}
		else {
			sumTopX += poleLine[0]; sumTopY += poleLine[1];
		}
		countFrame++;
		poleTop[0] = (int)(sumTopX / (float)countFrame);
		poleTop[1] = (int)(sumTopY / (float)countFrame);
		str_x = " x coordinates= " + to_string(poleX)+" pole Length: "+to_string(poleLength);
	}
	else {
		poleX = -1;
		poleLength = -1;
		str_x = "out of window";
	}

	putText(color_dst, str_x, Point(30, 60), FONT_HERSHEY_PLAIN, 1.0, Scalar(0, 255, 0));
	if (sucFlag) {
		cv::putText(color_dst, "Success", cv::Point(20, 300),
			0, 2, cv::Scalar(255, 255, 0));
		cv::putText(color_dst, "x="+to_string(shuttleXY_suc.x)+" y="+to_string(shuttleXY_suc.y), cv::Point(20, 350),
			0, 1, cv::Scalar(255, 255, 0));
	}
	if (countFrame > 580 && !sucFlag) {
		cv::putText(color_dst, "Fault", cv::Point(20, 300),
			0, 2, cv::Scalar(255, 255, 0));
	}
	
	imshow("Detected Lines", color_dst);
}

//ringtype=='g'�ŃS�[���f��
bool MyKinectV2::judgeCockThrough() {
	if (poleX != -1) {
		//�S�[���f���V���g���R�b�N�p
		if (ringtype == 'g') {
			ringSumB4 = ringSum;

			double ringRad = 400.0;
			double trueLength = 3000.0;
			int sideLength = (int)(poleLength * ringRad / trueLength);
			int x = abs(poleTop[0] - sideLength);
			int y = abs(poleTop[1] - sideLength*2);
			//�����O�͈̔͂�؂蔲��
			ringROI = cv::Rect(x, y, sideLength*2, sideLength*2);
			ringImage = depthImage(ringROI);
			//�����O�摜�Ƀt�B���^�[��������
			cv::filter2D(ringImage, filteredRing, -1, cv::Mat(::KERNELLENGTH, ::KERNELLENGTH, CV_64F, &kernel));
			//�t�B���^�[�̂Ȃ���臒l�ȉ��̒l���[���ɂ���
			for (int i = 0; i < filteredRing.total(); i++) {
				if (filteredRing.data[i] < FILTERTH) {
					filteredRing.data[i] = 0;
				}
			}
			ringSum = cv::sum(filteredRing);

			cv::imshow("filteredring", filteredRing);
			cv::imshow("GoldenringImage",ringImage);
			if (::SHATTLETH < ringSumB4[0]-ringSum[0]) {
				return true;
			}
			else return false;
		}

		else {
			ringSumB4 = ringSum;
			double ringRad = 400.0;
			double trueLength = 2000.0;
			int sideLength = (int)(poleLength * ringRad / trueLength);
			int x = abs(poleTop[0] - sideLength);
			int y = abs(poleTop[1] - sideLength * 2);
			//�����O�͈̔͂�؂蔲��
			ringROI = cv::Rect(x, y, sideLength*2, sideLength*2);
			ringImage = depthImage(ringROI);
			//�����O�摜�Ƀt�B���^�[��������
			cv::filter2D(ringImage, filteredRing, -1, cv::Mat(::KERNELLENGTH, ::KERNELLENGTH, CV_64F, &kernel));
			//�t�B���^�[�̂Ȃ���臒l�ȉ��̒l���[���ɂ���
			for (int i = 0; i < filteredRing.total(); i++) {
				if (filteredRing.data[i] < FILTERTH) {
					filteredRing.data[i] = 0;
				}
			}
			ringSum = cv::sum(filteredRing);
			std::cout << ringSum << std::endl;
			cv::imshow("filteredring", filteredRing);
			cv::imshow("ringImage", ringImage);
			if (::SHATTLETH < ringSumB4[0]-ringSum[0]) {
				return true;
			}
			else {
				//cv::filter2D(filteredRing, filteredRing, -1, cv::Mat(::KERNELLENGTH, ::KERNELLENGTH, CV_64F, &kernel));
				cv::minMaxLoc(filteredRing, NULL, NULL, NULL, &shuttleXY);
				tempImage = ringImage.clone();
				return false;
			}
		}
	}
	return false;
}

bool MyKinectV2::findShuttle() {
	cv::Mat color_ring;
	cv::cvtColor(tempImage, color_ring, CV_GRAY2BGR);
	// �V���g���R�b�N�̒��S��`��
	circle(color_ring, shuttleXY, 3, cv::Scalar(255, 0, 0), -1, 8, 0);
	//���S����̍������擾
	shuttleXY.x = (-ringImage.rows / 2 + shuttleXY.x);
	shuttleXY.y = (-ringImage.cols / 2 + shuttleXY.y);
	std::cout << "x= " << shuttleXY.x << std::endl << "y= " << shuttleXY.y << std::endl;
	float r = pow(ringImage.rows / 2.0, 2.0); 
	float d = pow(shuttleXY.x, 2.0) + pow(shuttleXY.y, 2.0);
	
	std::cout << r <<" "<< d << std::endl;

	if (d/r < SUCCESSTH) {
		shuttleXY_suc = shuttleXY;
		imshow("shuttlePoint", color_ring);

		return true;
	}
	else return false;
}

////�񂲂Ƃ̗v�f�̘a�̍ő�l��poleX�Ɋi�[
//void MyKinectV2::findPoleX() {
//	std::vector<int> depthColsSum(depthWidth);
//	for (int height = 0; height < depthImage.rows; height++) {
//		unsigned char* ptr=depthImage.ptr(height);
//		for (int width = 0; width < depthImage.cols; width++) {
//			depthColsSum[width] += ptr[width];
//		}
//	}
//	int maxItr = 0;
//	for (int width = 0; width < depthImage.cols; width++) {
//		depthColsSum[width] /= 255;
//		if (depthColsSum[maxItr] < depthColsSum[width]) {
//			maxItr = width;
//		}
//	}
//	if (depthColsSum[maxItr] > MINLINELENGTH) {
//		poleX = maxItr;
//	}
//	else {
//		poleX = -1;
//	}
//}
//void MyKinectV2::getHoughCircles(cv::Mat& src) {
//	static cv::Mat color_circle;
//	cvtColor(src, color_circle, CV_GRAY2BGR);
//
//	cv::HoughCircles(src, circles, CV_HOUGH_GRADIENT, 2, ::MINDIST, ::PARAM1, ::PARAM2, ::MINRADIUS, ::MAXRADIUS);
//	for (size_t i = 0; i < circles.size(); i++)
//	{
//		cv::Point center(cvRound(circles[i][0]), cvRound(circles[i][1]));
//		int radius = cvRound(circles[i][2]);
//		// �~�̒��S��`�悵�܂��D
//		circle(color_circle, center, 3, cv::Scalar(0, 255, 0), -1, 8, 0);
//		// �~��`�悵�܂��D
//		circle(color_circle, center, radius, cv::Scalar(0, 0, 255), 3, 8, 0);
//	}
//
//	if (circles.size() > 0) {
//		//�����O�̗̈��ringROI�Ɋi�[
//		ringROI = cv::Rect((int)(circles[0][0] - circles[0][2]), (int)(circles[0][1] - circles[0][2]),
//			circles[0][2] * 2, circles[0][2] * 2);
//		ringImage = depthImage(ringROI);
//		cv::imshow("ringImage", ringImage);
//	}
//
//	cv::namedWindow("circles", 1);
//	cv::imshow("circles", color_circle);
//
//
//}
