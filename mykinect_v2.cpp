#include"mykinect_v2.h"
namespace{	
//深度の範囲を指定
const UINT16 DEPTHRANGEMIN = 3500;
const UINT16 DEPTHRANGEMAX = 5000;
 
//動画の保存先と利用先
std::time_t now = std::time(nullptr);
std::string writeName = "./shuttleMovies/"+std::to_string(now)+".avi";
std::string useName = "./shuttleMovies/mis1.avi";

//houghLinesの精度
const int RHO=2; //ピクセル単位の投票空間の距離分解能
const int MINLINELENGTH=180; //これ以下の長さは棄却
const int MAXLINEGAP=10;  //2点が同一線分にあるとみなす最大距離
const int THRESHOLD=50;   //投票の閾値パラメータ
 
const double ANGLETHRESHOLD = 0.90;

//houghCirclesの精度
const int DP=2; //投票空間の分解能の比率
const int MINDIST=200;  //円の中心同士の距離
const int PARAM1=50;
const int PARAM2=90;	//投票数の閾値
const int MINRADIUS=0;  //円の半径の最小値
const int MAXRADIUS=0;  //円の半径の最大値

//シャトルコックの判定の閾値
const int FILTERTH = 150;  //フィルターを掛けた時にこれ以下の値は0にする
const int SHATTLETH=2000;  //シャトルコックが存在するか判定する閾値
const int KERNELLENGTH = 10; //カーネルの辺の長さ
double kernel[::KERNELLENGTH][::KERNELLENGTH];
const float SUCCESSTH = 0.9;  //リングの半径とシャトルコックの距離の比の閾値

std::vector<int> poleLine(4,-1);

//poleTopを更新するフレーム
const int updateFrame = 8;
long long countFrame=0;
}

MyKinectV2::MyKinectV2() {
	// デフォルトのKinectを取得する
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

// データの更新処理
void MyKinectV2::updateColor() {
	updateColorFrame();
}

// カラーフレームの更新
void MyKinectV2::updateColorFrame() {
	// フレームを取得する
	CComPtr<IColorFrame> colorFrame;
	auto ret = colorFrameReader->AcquireLatestFrame(&colorFrame);
	if (FAILED(ret)) {
		return;
	}

	// 指定の形式でデータを取得する
	ERROR_CHECK(colorFrame->CopyConvertedFrameDataToArray(
		colorBuffer.size(), &colorBuffer[0], colorFormat));
}

void MyKinectV2::drawColor() {
	drawColorFrame();
}

//ここに処理を書く
void MyKinectV2::drawColorFrame() {
#if 0
	// カラーデータを表示する
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

	//マウスの初期位置(別になんでも良い)
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

void MyKinectV2::drawDepth() {
	drawDepthFrame();
}

//映像の保存
void MyKinectV2::saveDepthMovie() {
	static cv::VideoWriter writer(writeName, cv::VideoWriter::fourcc('X', 'V', 'I', 'D'), 30, cv::Size(depthWidth, depthHeight), false);
	// Depthデータを表示する
	depthImage = cv::Mat(depthHeight, depthWidth, CV_8UC1);

	//指定した距離のみ2値化してdepthImageに格納
	for (int i = 0; i < depthImage.total(); ++i) {
		if (DEPTHRANGEMIN < depthBuffer[i] && depthBuffer[i] < DEPTHRANGEMAX) {
			//depthImage.data[i] = depthBuffer[i] % 255;
			//depthImage.data[i] = (unsigned char)(((float)depthBuffer[i] / 8000.0)* 255.0);
			depthImage.data[i] = 255;  //2値化
		}
		else {
			depthImage.data[i] = 0;
		}
	}
	writer << depthImage;
	imshow("depthImage", depthImage);
}

//映像を使って処理（デバッグ用）
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

		//平滑化
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

//得られた距離はdepthBufferに格納される
//ここに処理を書く
void MyKinectV2::drawDepthFrame() {
	// Depthデータを表示する
	depthImage=cv::Mat(depthHeight, depthWidth, CV_8UC1);
		//指定した距離のみ2値化してdepthImageに格納
	for (int i = 0; i < depthImage.total(); ++i) {
		if (DEPTHRANGEMIN < depthBuffer[i] && depthBuffer[i] < DEPTHRANGEMAX) {
			//depthImage.data[i] = depthBuffer[i] % 255;
			//depthImage.data[i] = (unsigned char)(((float)depthBuffer[i] / 8000.0)* 255.0);
			depthImage.data[i] = 255;  //2値化
		}
		else {
			depthImage.data[i] = 0;
		}
	}

	//平滑化
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
	// Depthデータのインデックスを取得して、その場所の距離を表示する
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

	//canny変換
	Canny(src, dst, 50, 200, 3);
	cvtColor(dst, color_dst, CV_GRAY2BGR);
	HoughLinesP(dst, lines, ::RHO, CV_PI / 360.0, ::THRESHOLD, ::MINLINELENGTH, ::MAXLINEGAP);

	std::vector<cv::Vec4i> verticalLines;
	std::vector<double> verticalLength;

	//一定以上の角度のものを抽出
	for (size_t i = 0; i < lines.size(); i++) {
		double angle = atan(abs(lines[i][1] - lines[i][3]) / (abs(lines[i][0] - lines[i][2]) + 1e-10));
		if (lines[i][0] < 507 && lines[i][0]>5) { //端っこがバグの原因になるから
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
		//一番長い線を見つける
		if (verticalLength[longestItr] <= verticalLength[i]) {
			longestLength = verticalLength[i];
			longestItr = i;
		}
	}

	//lineが見つかっている場合の処理
	if (verticalLines.size() > 0) {
		//一番長い線をpoleLineに入れる updateFrameフレームおき
		if (countFrame % updateFrame == 0) {
			for (int i = 0; i < 4; i++) {
				poleLine[i] = verticalLines[longestItr][i];
			}
			poleLength = (int)longestLength;
		}
	}
	
	if(poleLine[0]>0){
		//poleLineを描画（デバッグ用）
		line(color_dst, Point(poleLine[0], poleLine[1]), Point(poleLine[2], poleLine[3]), Scalar(0, 0, 255), 3, 8);
		poleX = poleLine[0];
		//poleの頂点の座標を格納
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

//ringtype=='g'でゴールデン
bool MyKinectV2::judgeCockThrough() {
	if (poleX != -1) {
		//ゴールデンシャトルコック用
		if (ringtype == 'g') {
			ringSumB4 = ringSum;

			double ringRad = 400.0;
			double trueLength = 3000.0;
			int sideLength = (int)(poleLength * ringRad / trueLength);
			int x = abs(poleTop[0] - sideLength);
			int y = abs(poleTop[1] - sideLength*2);
			//リングの範囲を切り抜く
			ringROI = cv::Rect(x, y, sideLength*2, sideLength*2);
			ringImage = depthImage(ringROI);
			//リング画像にフィルターをかける
			cv::filter2D(ringImage, filteredRing, -1, cv::Mat(::KERNELLENGTH, ::KERNELLENGTH, CV_64F, &kernel));
			//フィルターのなかで閾値以下の値をゼロにする
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
			//リングの範囲を切り抜く
			ringROI = cv::Rect(x, y, sideLength*2, sideLength*2);
			ringImage = depthImage(ringROI);
			//リング画像にフィルターをかける
			cv::filter2D(ringImage, filteredRing, -1, cv::Mat(::KERNELLENGTH, ::KERNELLENGTH, CV_64F, &kernel));
			//フィルターのなかで閾値以下の値をゼロにする
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
	// シャトルコックの中心を描画
	circle(color_ring, shuttleXY, 3, cv::Scalar(255, 0, 0), -1, 8, 0);
	//中心からの差分を取得
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

////列ごとの要素の和の最大値をpoleXに格納
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
//		// 円の中心を描画します．
//		circle(color_circle, center, 3, cv::Scalar(0, 255, 0), -1, 8, 0);
//		// 円を描画します．
//		circle(color_circle, center, radius, cv::Scalar(0, 0, 255), 3, 8, 0);
//	}
//
//	if (circles.size() > 0) {
//		//リングの領域をringROIに格納
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
