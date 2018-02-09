#include"NHKdebug.h"

//動画の保存先と利用先
std::time_t now = std::time(nullptr);
std::string writeName_depth = "./shuttleMovies/depth"+ std::to_string(now) + ".avi";
std::string writeName_RGB = "./shuttleMovies/RGB" + std::to_string(now) + ".avi";
std::string useName = "./shuttleMovies/suc1.avi";

void KinectDebug::initializeDepth() {
	// Depthリーダーを取得する
	CComPtr<IDepthFrameSource> depthFrameSource;
	ERROR_CHECK(kinect->get_DepthFrameSource(&depthFrameSource));
	ERROR_CHECK(depthFrameSource->OpenReader(&depthFrameReader));

	// Depth画像のサイズを取得する
	CComPtr<IFrameDescription> depthFrameDescription;
	ERROR_CHECK(depthFrameSource->get_FrameDescription(
		&depthFrameDescription));
	ERROR_CHECK(depthFrameDescription->get_Width(&depthWidth));
	ERROR_CHECK(depthFrameDescription->get_Height(&depthHeight));
	//マウスの初期位置
	depthPointX = depthWidth / 2;
	depthPointY = depthHeight / 2;

	// Depthの最大値、最小値を取得する
	UINT16 minDepthReliableDistance;
	UINT16 maxDepthReliableDistance;
	ERROR_CHECK(depthFrameSource->get_DepthMinReliableDistance(
		&minDepthReliableDistance));
	ERROR_CHECK(depthFrameSource->get_DepthMaxReliableDistance(
		&maxDepthReliableDistance));
	std::cout << "Depth最小値       : " << minDepthReliableDistance << std::endl;
	std::cout << "Depth最大値       : " << maxDepthReliableDistance << std::endl;
	std::cout << "depthWidth: " << depthWidth << std::endl;
	std::cout << "depthHeight: " << depthHeight << std::endl;

	// バッファーを作成する
	depthBuffer.resize(depthWidth * depthHeight);

	// マウスクリックのイベントを登録する
	cv::namedWindow(DepthWindowName);
	cv::setMouseCallback(DepthWindowName, &KinectDebug::mouseCallback, this);

}

void KinectDebug::saveDepthMovie() {
	static cv::VideoWriter depthWriter(writeName_depth,cv::VideoWriter::fourcc('X', 'V', 'I', 'D'), 30, cv::Size(depthWidth, depthHeight), false);
	setDepth();
	depthWriter << depthImage;
	imshow("depthImage", depthImage);
}

void KinectDebug::saveRGBMovie() {
	using namespace cv;
	Size sz(640, 480);
	Mat img;
	static  cv::VideoWriter rgbWriter(writeName_RGB, cv::VideoWriter::fourcc('X', 'V', 'I', 'D'), 30, cv::Size(colorWidth, colorHeight), true);
	setRGB();
	cvtColor(RGBImage, img, CV_BGRA2BGR);
	rgbWriter << img;
	imshow("RGBImage", RGBImage);
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

void KinectDebug::showRGB() {
	cv::Mat img;
	cv::resize(RGBImage, img, cv::Size(), 0.3, 0.3);
	cv::imshow("RGB", img);
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
	std::vector<cv::Vec4i> lines;
	string str_x;
	Canny(src, dst, 50, 200, 3);
	cvtColor(dst, color_dst, CV_GRAY2BGR);
	HoughLinesP(dst, lines, RHO, CV_PI / 360.0, THRESHOLD, MINLINELENGTH, MAXLINEGAP);

	auto v = [](Pole m1, Pole m2) {return m1.length < m2.length; };
	priority_queue<Pole, std::vector<Pole>, decltype(v)> Poles(v);
	//一定以上の角度のものを抽出
	for (size_t i = 0; i < lines.size(); i++) {
		double angle = atan(abs(lines[i][1] - lines[i][3]) / (abs(lines[i][0] - lines[i][2]) + 1e-10));
		if (lines[i][0] < 507 && lines[i][0]>5) { //端っこがバグの原因になるから
			if (angle > CV_PI / 2.0*ANGLETHRESHOLD) {
				int length = (int)sqrt(pow((lines[i][0] - lines[i][2]), 2) + pow((lines[i][1] - lines[i][3]), 2));
				Poles.push(Pole(length, lines[i]));
			}
		}
	}

	//poleLineとpoleLine.topを更新
	if (!Poles.empty()) {
		if (countFrame % updateFrame == 0) {
			poleLine = Poles.top();
			if (poleLine.line[1] >= poleLine.line[3]) { poleLine.top[0] = poleLine.line[2]; poleLine.top[1] += poleLine.line[3]; }
			else { poleLine.top[0] = poleLine.line[0]; poleLine.top[1] += poleLine.line[1]; }
		}
	}

	//poleLineを描画（デバッグ用）
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
	poleLine.ringtype == 'g' ? (ringRad = 400.0, trueLength = 3000.0) : (ringRad = 400.0, trueLength = 2000.0);
	if (poleLine.top[0] != -1) {
		int sideLength = (int)(poleLine.length * ringRad / trueLength);
		int x = abs(poleLine.top[0] - sideLength), y = abs(poleLine.top[1] - sideLength * 2);
		poleLine.ringROI = cv::Rect(x, y, sideLength * 2, sideLength * 2);
		poleLine.ringImage = depthImage(poleLine.ringROI);
		cv::filter2D(poleLine.ringImage, poleLine.ringImage, -1,kernel);
		cv::threshold(poleLine.ringImage, poleLine.ringImage, FILTERTH, 0, cv::THRESH_TOZERO);
		cv::imshow("poleLine.ringImage", poleLine.ringImage);
		if (SHATTLETH < ringSum[0] - cv::sum(poleLine.ringImage)[0]) {
			return true;
		}
		else {
			cv::minMaxLoc(poleLine.ringImage, NULL, NULL, NULL, &poleLine.shuttleXY);
			ringSum = cv::sum(poleLine.ringImage);
			return false;
		}
	}
	return false;
}

bool KinectDebug::findShuttle() {
	cv::Mat color_ring;
	//シャトルコックの中心を描画
	//circle(color_ring, poleLine.shuttleXY, 3, cv::Scalar(255, 0, 0), -1, 8, 0);
	poleLine.shuttleXY.x = (-poleLine.ringImage.rows / 2 + poleLine.shuttleXY.x);
	poleLine.shuttleXY.y = (-poleLine.ringImage.cols / 2 + poleLine.shuttleXY.y);
	std::cout << "x= " << poleLine.shuttleXY.x << std::endl << "y= " << poleLine.shuttleXY.y << std::endl;
	float r = pow(poleLine.ringImage.rows / 2.0, 2.0);
	float d = pow(poleLine.shuttleXY.x, 2.0) + pow(poleLine.shuttleXY.y, 2.0);

	std::cout << r << " " << d << std::endl;

	if (d / r < SUCCESSTH) {
		poleLine.shuttleXY_suc = poleLine.shuttleXY;
		//imshow("shuttlePoint", color_ring);
		return true;
	}
	else return false;
}