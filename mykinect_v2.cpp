#include"mykinect_v2.h"

MyKinectV2::MyKinectV2() {
	// デフォルトのKinectを取得する
	ERROR_CHECK(::GetDefaultKinectSensor(&kinect));
	ERROR_CHECK(kinect->Open());
}

MyKinectV2::~MyKinectV2()
{
	// Kinectの動作を終了する
	if (kinect != nullptr) {
		kinect->Close();
	}
}

void MyKinectV2::initializeColor() {
	// カラーリーダーを取得する
	CComPtr<IColorFrameSource> colorFrameSource;
	ERROR_CHECK(kinect->get_ColorFrameSource(&colorFrameSource));
	ERROR_CHECK(colorFrameSource->OpenReader(&colorFrameReader));

	// デフォルトのカラー画像のサイズを取得する
	CComPtr<IFrameDescription> defaultColorFrameDescription;
	ERROR_CHECK(colorFrameSource->get_FrameDescription(&defaultColorFrameDescription));
	ERROR_CHECK(defaultColorFrameDescription->get_Width(&colorWidth));
	ERROR_CHECK(defaultColorFrameDescription->get_Height(&colorHeight));
	ERROR_CHECK(defaultColorFrameDescription->get_BytesPerPixel(&colorBytesPerPixel));
	std::cout << "default : " << colorWidth << ", " << colorHeight << ", " << colorBytesPerPixel << std::endl;

	// カラー画像のサイズを取得する
	CComPtr<IFrameDescription> colorFrameDescription;
	ERROR_CHECK(colorFrameSource->CreateFrameDescription(
		colorFormat, &colorFrameDescription));
	ERROR_CHECK(colorFrameDescription->get_Width(&colorWidth));
	ERROR_CHECK(colorFrameDescription->get_Height(&colorHeight));
	ERROR_CHECK(colorFrameDescription->get_BytesPerPixel(&colorBytesPerPixel));
	std::cout << "create  : " << colorWidth << ", " << colorHeight << ", " << colorBytesPerPixel << std::endl;

	// バッファーを作成する
	colorBuffer.resize(colorWidth * colorHeight * colorBytesPerPixel);
}

void MyKinectV2::updateColorFrame() {
	// フレームを取得する
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

void MyKinectV2::updateDepthFrame() {
	// Depthフレームを取得する
	CComPtr<IDepthFrame> depthFrame;
	auto ret = depthFrameReader->AcquireLatestFrame(&depthFrame);
	if (ret != S_OK) {
		return;
	}

	// データを取得する
	ERROR_CHECK(depthFrame->CopyFrameDataToArray(
		depthBuffer.size(), &depthBuffer[0]));
}

void MyKinectV2::initializeDepth() {
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
	// Depthの最大値、最小値を取得する
	UINT16 minDepthReliableDistance;
	UINT16 maxDepthReliableDistance;
	ERROR_CHECK(depthFrameSource->get_DepthMinReliableDistance(
		&minDepthReliableDistance));
	ERROR_CHECK(depthFrameSource->get_DepthMaxReliableDistance(
		&maxDepthReliableDistance));

	// バッファーを作成する
	depthBuffer.resize(depthWidth * depthHeight);
}
void MyKinectV2::setDepth() {
	updateDepthFrame();
	depthImage = cv::Mat(depthHeight, depthWidth, CV_8UC1);
	for (int i = 0; i < depthImage.total(); i++) {
		depthImage.data[i] = depthBuffer[i];
	}
}

void MyKinectV2::getShuttleLoc() {
	setDepth();
	binarization(depthImage, MINDEPTH, MAXDEPTH);
	cv::medianBlur(depthImage, depthImage, 3);
	getHoughLines(depthImage);
	if (judgeCockThrough()) {
		if (findShuttle()) {
			std::cout << "success!" << std::endl;
		}
	}
}

void MyKinectV2::binarization(cv::Mat image, const int minDepth, const int maxDepth) {
	for (int i = 0; i < image.total(); ++i) {
		if (minDepth < image.data[i] && image.data[i] < maxDepth) {
			image.data[i] = 255;  //2値化
		}
		else {
			image.data[i] = 0;
		}
	}
}


void MyKinectV2::getHoughLines(cv::Mat& src) {
	using namespace std;
	using namespace cv;

	static Mat dst;

	Canny(src, dst, 50, 200, 3);
	HoughLinesP(dst, lines, RHO, CV_PI / 360.0, THRESHOLD, MINLINELENGTH, MAXLINEGAP);
	
	auto v = [](vLine m1, vLine m2) {return m1.length < m2.length; };
	priority_queue<vLine,std::vector<vLine>,decltype(v)> vLines(v);
	//一定以上の角度のものを抽出
	for (size_t i = 0; i < lines.size(); i++) {
		double angle = atan(abs(lines[i][1] - lines[i][3]) / (abs(lines[i][0] - lines[i][2]) + 1e-10));
		if (lines[i][0] < 507 && lines[i][0]>5) { //端っこでバグる
			if (angle > CV_PI / 2.0*ANGLETHRESHOLD) {
				int length = (int)sqrt(pow((lines[i][0] - lines[i][2]), 2) + pow((lines[i][1] - lines[i][3]), 2));
				vLines.push(vLine(length,lines[i]));
			}
		}
	}

	//poleLineとpoleLine.topへ代入
	if (!vLines.empty()) {
		if (countFrame % updateFrame == 0) {
			poleLine = vLines.top();
			if (poleLine.line[1] >= poleLine.line[3]) {  poleLine.top[0] = poleLine.line[2]; poleLine.top[1] += poleLine.line[3]; }
			else { poleLine.top[0] = poleLine.line[0]; poleLine.top[1] += poleLine.line[1]; }
		}
	}
	countFrame++;
}

bool MyKinectV2::judgeCockThrough() {
	//static auto kernel = makeKernel(10);
	static cv::Scalar ringSum(-1e8);
	double ringRad, trueLength;
	ringtype == 'g' ? (ringRad = 400.0, trueLength = 3000.0) : (ringRad = 400.0, trueLength = 2000.0);
	if (poleLine.top[0] != -1) {
		int sideLength = (int)(poleLine.length * ringRad / trueLength);
		int x = abs(poleLine.top[0] - sideLength), y = abs(poleLine.top[1] - sideLength * 2);
		//リングの範囲を切り抜く
		ringROI = cv::Rect(x, y, sideLength * 2, sideLength * 2);
		ringImage = depthImage(ringROI);
		//フィルターをかける
	//	cv::filter2D(ringImage, ringImage, -1, cv::Mat(KERNELLENGTH, KERNELLENGTH, CV_64F, &kernel));
		cv::threshold(ringImage, ringImage, FILTERTH, 0, cv::THRESH_TOZERO);

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

bool MyKinectV2::findShuttle() {
	//中心からの差分を取得
	shuttleXY.x = (-ringImage.rows / 2 + shuttleXY.x); shuttleXY.y = (-ringImage.cols / 2 + shuttleXY.y);
	float r = pow(ringImage.rows / 2.0, 2.0), d = pow(shuttleXY.x, 2.0) + pow(shuttleXY.y, 2.0);

	std::cout << "x= " << shuttleXY.x << std::endl << "y= " << shuttleXY.y << std::endl;
	std::cout << r <<" "<< d << std::endl;

	if (d/r < SUCCESSTH) {
		shuttleXY_suc = shuttleXY;
		return true;
	}
	else return false;
}
