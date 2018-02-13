#include"mykinect_v2.h"

MyKinectV2::MyKinectV2() {
	// �f�t�H���g��Kinect���擾����
	ERROR_CHECK(::GetDefaultKinectSensor(&kinect));
	ERROR_CHECK(kinect->Open());
	// Retrieved Coordinate Mapper
	ERROR_CHECK(kinect->get_CoordinateMapper(&coordinateMapper));
}

MyKinectV2::~MyKinectV2()
{
	// Kinect�̓�����I������
	if (kinect != nullptr) {
		kinect->Close();
	}
}

#pragma region get_image
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
	std::cout << "colorWidth : " << colorWidth << std::endl << "colorHeight: " << colorHeight << std::endl << "colorBytesPerPixel: " << colorBytesPerPixel << std::endl;

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

void MyKinectV2::updateColorFrame() {
	// �t���[�����擾����
	CComPtr<IColorFrame> colorFrame;
	auto ret = colorFrameReader->AcquireLatestFrame(&colorFrame);
	if (FAILED(ret)) {
		return;
	}
	ERROR_CHECK(colorFrame->CopyConvertedFrameDataToArray(
		colorBuffer.size(), &colorBuffer[0], colorFormat));
}

void MyKinectV2::setRGB() {
	updateColorFrame();
	RGBImage = cv::Mat(colorHeight, colorWidth, CV_8UC4, &colorBuffer[0]);
}

void MyKinectV2::setMappedRGB() {
	updateColorFrame();
	// Retrieve Mapped Coordinates
	std::vector<ColorSpacePoint> colorSpacePoints(depthWidth * depthHeight);
	ERROR_CHECK(coordinateMapper->MapDepthFrameToColorSpace(depthBuffer.size(), &depthBuffer[0], colorSpacePoints.size(), &colorSpacePoints[0]));
	// Mapped Color Buffer
	std::vector<BYTE> buffer(depthWidth * depthHeight * colorBytesPerPixel);

	// Mapping Color Data to Depth Resolution
	for (int depthY = 0; depthY < depthHeight; depthY++) {
		for (int depthX = 0; depthX < depthWidth; depthX++) {
			const unsigned int depthIndex = depthY * depthWidth + depthX;
			const int colorX = static_cast<int>(colorSpacePoints[depthIndex].X + 0.5f);
			const int colorY = static_cast<int>(colorSpacePoints[depthIndex].Y + 0.5f);
			if ((0 <= colorX) && (colorX < colorWidth) && (0 <= colorY) && (colorY < colorHeight)) {
				const unsigned int colorIndex = colorY * colorWidth + colorX;
				buffer[depthIndex * colorBytesPerPixel + 0] = colorBuffer[colorIndex * colorBytesPerPixel + 0];
				buffer[depthIndex * colorBytesPerPixel + 1] = colorBuffer[colorIndex * colorBytesPerPixel + 1];
				buffer[depthIndex * colorBytesPerPixel + 2] = colorBuffer[colorIndex * colorBytesPerPixel + 2];
				buffer[depthIndex * colorBytesPerPixel + 3] = colorBuffer[colorIndex * colorBytesPerPixel + 3];
			}
		}
	}

	RGBImage = cv::Mat(depthHeight, depthWidth, CV_8UC4, &buffer[0]).clone();
}

bool MyKinectV2::setRGBbyMovie(std::string movieName) {
	static cv::VideoCapture cap(movieName);
	cv::Mat temp;
	cap >> temp;
	if (temp.empty()) return false;
	cv::cvtColor(temp, RGBImage, CV_BGR2BGRA);
	return true;
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
	//�}�E�X�̏����ʒu
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
#pragma endregion

	// �o�b�t�@�[���쐬����
	depthBuffer.resize(depthWidth * depthHeight);
	// �}�E�X�N���b�N�̃C�x���g��o�^����
	cv::setMouseCallback("depthImage", mouseCallback, this);

}

void MyKinectV2::setDepth() {
	updateDepthFrame();
	depthImage = cv::Mat(depthHeight, depthWidth, CV_8UC1);
	for (int i = 0; i < depthImage.total(); i++) {
		depthImage.data[i] = (UINT16)((depthBuffer[i] / 8000.0) * 255.0);
	}
}

bool MyKinectV2::setDepthbyMovie(std::string movieName) {
	static cv::VideoCapture cap(movieName);
	cv::Mat temp;
	cap >> temp;
	if (temp.empty()) return false;
	cv::cvtColor(temp, depthImage, CV_RGB2GRAY);
	return true;
}
#pragma endregion

void MyKinectV2::showDistance() {
	int index = (depthPointY * depthWidth) + depthPointX;
	std::stringstream ss;
	ss << depthBuffer[index] << "mm";
	cv::Mat dst;
	cv::circle(depthImage, cv::Point(depthPointX, depthPointY), 10,
		cv::Scalar(0, 0, 255), 2);
	cv::putText(depthImage, ss.str(), cv::Point(depthPointX, depthPointY),
		0, 1, cv::Scalar(255, 255, 255));
	cv::imshow("depthImage", depthImage);
}