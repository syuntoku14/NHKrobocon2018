#include"NHKdebug.h"

//����̕ۑ���Ɨ��p��
std::time_t now = std::time(nullptr);
std::string writeName = "./shuttleMovies/" + std::to_string(now) + ".avi";
std::string useName = "./shuttleMovies/suc1.avi";

void KinectDebug::initializeDepth() {
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

	// �o�b�t�@�[���쐬����
	depthBuffer.resize(depthWidth * depthHeight);

	// �}�E�X�N���b�N�̃C�x���g��o�^����
	cv::namedWindow(DepthWindowName);
	cv::setMouseCallback(DepthWindowName, &KinectDebug::mouseCallback, this);

}

void KinectDebug::saveDepthMovie() {
	static cv::VideoWriter writer(writeName,cv::VideoWriter::fourcc('X', 'V', 'I', 'D'), 30, cv::Size(depthWidth, depthHeight), false);
	setDepth();
	writer << depthImage;
	imshow("depthImage", depthImage);
}

void KinectDebug::useDepthMovie() {
	static cv::VideoCapture cap(useName);
	cv::Mat temp;

	while (1) {
		cap >> temp;
		if (temp.empty()) break;
		auto key = cv::waitKey(1);
		if (key == 'q') break;
		else if (key == 's') cv::waitKey(0);
		cv::cvtColor(temp, depthImage, CV_RGB2GRAY);
		//binarization(depthImage, MINDEPTH, MAXDEPTH);
		cv::medianBlur(depthImage, depthImage, 3);
		showDistance();
		getHoughLines(depthImage);
		if (judgeCockThrough()) {
			if (findShuttle()) {
				std::cout << "success!" << std::endl;
			}
		}
		if (countFrame > 530) {
			std::cout << "Fault" << std::endl;
		}
	}
}

void KinectDebug::showDistance() {
	int index = (depthPointY * depthWidth) + depthPointX;
	std::stringstream ss;
	ss << depthBuffer[index] << "mm";
	cv::Mat dst;

	cv::circle(depthImage, cv::Point(depthPointX, depthPointY), 10,
		cv::Scalar(0, 0, 255), 2);
	cv::putText(depthImage, ss.str(), cv::Point(depthPointX, depthPointY),
		0, 1, cv::Scalar(255, 255, 255));
	cv::imshow(DepthWindowName, depthImage);
}

void KinectDebug::getHoughLines(cv::Mat& src) {
	using namespace std;
	using namespace cv;

	static Mat dst, color_dst;
	string str_x;
	Canny(src, dst, 50, 200, 3);
	cvtColor(dst, color_dst, CV_GRAY2BGR);
	HoughLinesP(dst, lines, RHO, CV_PI / 360.0, THRESHOLD, MINLINELENGTH, MAXLINEGAP);

	auto v = [](vLine m1, vLine m2) {return m1.length < m2.length; };
	priority_queue<vLine, std::vector<vLine>, decltype(v)> vLines(v);
	//���ȏ�̊p�x�̂��̂𒊏o
	for (size_t i = 0; i < lines.size(); i++) {
		double angle = atan(abs(lines[i][1] - lines[i][3]) / (abs(lines[i][0] - lines[i][2]) + 1e-10));
		if (lines[i][0] < 507 && lines[i][0]>5) { //�[�������o�O�̌����ɂȂ邩��
			if (angle > CV_PI / 2.0*ANGLETHRESHOLD) {
				int length = (int)sqrt(pow((lines[i][0] - lines[i][2]), 2) + pow((lines[i][1] - lines[i][3]), 2));
				vLines.push(vLine(length, lines[i]));
			}
		}
	}

	//poleLine��poleLine.top���X�V
	if (!vLines.empty()) {
		if (countFrame % updateFrame == 0) {
			poleLine = vLines.top();
			if (poleLine.line[1] >= poleLine.line[3]) { poleLine.top[0] = poleLine.line[2]; poleLine.top[1] += poleLine.line[3]; }
			else { poleLine.top[0] = poleLine.line[0]; poleLine.top[1] += poleLine.line[1]; }
		}
	}

	//poleLine��`��i�f�o�b�O�p�j
	if (poleLine.length > 0) { 
		str_x = " x coordinates= " + to_string(poleLine.line[0]) + " pole Length: " + to_string(poleLine.length);
		line(color_dst, Point(poleLine.line[0], poleLine.line[1]), Point(poleLine.line[2], poleLine.line[3]), Scalar(0, 0, 255), 3, 8); 
	}
	else str_x = "out of window";
	putText(color_dst, str_x, Point(30, 60), FONT_HERSHEY_PLAIN, 1.0, Scalar(0, 255, 0));
	imshow("Detected Lines", color_dst);
}

bool KinectDebug::judgeCockThrough() {
	static cv::Mat kernel = cv::Mat::ones(KERNELSIZE, KERNELSIZE, CV_64F) / (double)(KERNELSIZE*KERNELSIZE);
	static cv::Scalar ringSum(-1e8);
	double ringRad, trueLength;
	ringtype == 'g' ? (ringRad = 400.0, trueLength = 3000.0) : (ringRad = 400.0, trueLength = 2000.0);
	if (poleLine.top[0] != -1) {
		int sideLength = (int)(poleLine.length * ringRad / trueLength);
		int x = abs(poleLine.top[0] - sideLength), y = abs(poleLine.top[1] - sideLength * 2);
		ringROI = cv::Rect(x, y, sideLength * 2, sideLength * 2);
		ringImage = depthImage(ringROI);
		cv::filter2D(ringImage, ringImage, -1,kernel);
		cv::threshold(ringImage, ringImage, FILTERTH, 0, cv::THRESH_TOZERO);
		cv::imshow("ringImage", ringImage);
		if (SHATTLETH < ringSum[0] - cv::sum(ringImage)[0]) {
			return true;
		}
		else {
			cv::minMaxLoc(ringImage, NULL, NULL, NULL, &shuttleXY);
			ringSum = cv::sum(ringImage);
			return false;
		}
	}
	return false;
}

bool KinectDebug::findShuttle() {
	cv::Mat color_ring;
	cv::cvtColor(tempImage, color_ring, CV_GRAY2BGR);
	//�V���g���R�b�N�̒��S��`��
	//circle(color_ring, shuttleXY, 3, cv::Scalar(255, 0, 0), -1, 8, 0);
	shuttleXY.x = (-ringImage.rows / 2 + shuttleXY.x);
	shuttleXY.y = (-ringImage.cols / 2 + shuttleXY.y);
	std::cout << "x= " << shuttleXY.x << std::endl << "y= " << shuttleXY.y << std::endl;
	float r = pow(ringImage.rows / 2.0, 2.0);
	float d = pow(shuttleXY.x, 2.0) + pow(shuttleXY.y, 2.0);

	std::cout << r << " " << d << std::endl;

	if (d / r < SUCCESSTH) {
		shuttleXY_suc = shuttleXY;
		//imshow("shuttlePoint", color_ring);
		return true;
	}
	else return false;
}