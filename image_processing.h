#pragma once
#include"MyKinectV2.h"
#include"xmlManage.h"

void binarization(cv::Mat& image, const int minDepth, const int maxDepth) {
	double mid = 255 * minDepth / 8000.0, mxd = 255 * maxDepth / 8000.0;
	for (int i = 0; i < image.total(); ++i) {
		if (mid < image.data[i] && image.data[i] < mxd) {
			image.data[i] = 255;  //2ílâª
		}
		else {
			image.data[i] = 0;
		}
	}
};

class PoleData {
public:
	char ringtype;
	int length;
	cv::Vec4i poleLine;
	cv::Vec4f poleLine_f;
	cv::Vec2i topPosition;
	cv::Rect ringROI;
	cv::Mat ringImage;
	cv::Point shuttleXY;

	PoleData(int length, cv::Vec4i line) {
		this->length = length;
		this->poleLine = line;
	};
	PoleData(int length, cv::Vec4f line) {
		this->length = length;
		this->poleLine_f = line;
		for (int i = 0; i < 4; i++) {
			poleLine[i] = (int)poleLine_f[i];
		}
	};
	PoleData() { this->length = -1; };
};

class HoughLineParamaters {
public:
	int rho, threshold, minLineLength, maxLineGap;
	double theta, angleThreshold;
	HoughLineParamaters(int rho,double theta,int threshold,int minLineLength,int maxLineGap, double angleThreshold) {
		this->rho = rho; this->theta = theta; this->threshold = threshold;
		this->minLineLength = minLineLength; this->maxLineGap = maxLineGap;
		this->angleThreshold = angleThreshold;
	};
};

void setPoleDatabyHoughLine(cv::Mat& src, PoleData &poledata, HoughLineParamaters params) {
	using namespace std;
	using namespace cv;

	Mat dst;
	vector<Vec4i> lines;
	Canny(src, dst, 50, 200, 3);
	HoughLinesP(dst, lines, params.rho, params.theta, params.threshold, params.minLineLength, params.maxLineGap);

	auto v = [](PoleData m1, PoleData m2) {return m1.length < m2.length; };
	priority_queue<PoleData, std::vector<PoleData>, decltype(v)> poles(v);

	//àÍíËà»è„ÇÃäpìxÇÃÇ‡ÇÃÇíäèo
	for (size_t i = 0; i < lines.size(); i++) {
		double angle = atan(abs(lines[i][1] - lines[i][3]) / (abs(lines[i][0] - lines[i][2]) + 1e-10));
		if (lines[i][0] < 507 && lines[i][0]>5) { //í[Ç¡Ç±Ç≈ÉoÉOÇÈ
			if (angle > CV_PI / 2.0*params.angleThreshold) {
				int length = (int)sqrt(pow((lines[i][0] - lines[i][2]), 2) + pow((lines[i][1] - lines[i][3]), 2));
				poles.push(PoleData(length, lines[i]));
			}
		}
	}

	if (!poles.empty()) {
		//í∑Ç≥Ç™ç≈ëÂÇÃlineÇpoledataÇ…äiî[
		poledata = poles.top();
		//í∏ì_èÓïÒÇäiî[
		if (poledata.poleLine[1] >= poledata.poleLine[3]) { poledata.topPosition[0] = poledata.poleLine[2]; poledata.topPosition[1] += poledata.poleLine[3]; }
		else { poledata.topPosition[0] = poledata.poleLine[0]; poledata.topPosition[1] += poledata.poleLine[1]; }
	}

};

void setPoleDatabyLSD(cv::Mat &img, PoleData &poledata, double angleThreshold) {
	using namespace cv;
	using namespace std;
	Mat dst;
	Canny(img, dst, 50, 200, 3);
	Ptr<LineSegmentDetector> ls = createLineSegmentDetector(LSD_REFINE_STD);
	vector<Vec4f> lines;
	ls->detect(dst, lines);

	auto v = [](PoleData m1, PoleData m2) {return m1.length < m2.length; };
	priority_queue<PoleData, std::vector<PoleData>, decltype(v)> poles(v);

	//àÍíËà»è„ÇÃäpìxÇÃÇ‡ÇÃÇíäèo
	for (size_t i = 0; i < lines.size(); i++) {
		double angle = atan(abs(lines[i][1] - lines[i][3]) / (abs(lines[i][0] - lines[i][2]) + 1e-10));
		if (lines[i][0] < 507 && lines[i][0]>5) { //í[Ç¡Ç±Ç≈ÉoÉOÇÈ
			if (angle > CV_PI / 2.0*angleThreshold) {
				int length = (int)sqrt(pow((lines[i][0] - lines[i][2]), 2) + pow((lines[i][1] - lines[i][3]), 2));
				poles.push(PoleData(length, lines[i]));
			}
		}
	}

	if (!poles.empty()) {
		//í∑Ç≥Ç™ç≈ëÂÇÃlineÇpoledataÇ…äiî[
		poledata = poles.top();
		//í∏ì_èÓïÒÇäiî[
		if (poledata.poleLine[1] >= poledata.poleLine[3]) { poledata.topPosition[0] = poledata.poleLine[2]; poledata.topPosition[1] += poledata.poleLine[3]; }
		else { poledata.topPosition[0] = poledata.poleLine[0]; poledata.topPosition[1] += poledata.poleLine[1]; }
	}
};

void showPoleLine(cv::Mat &img, PoleData poledata) {
	using namespace std;
	using namespace cv;
	Mat color_dst;
	string str_x;
	cvtColor(img, color_dst, CV_GRAY2BGR);
	if (poledata.length > 0) {
		str_x=" x coordinates= " + to_string(poledata.poleLine[0]) + " pole Length: " + to_string(poledata.length);
		line(color_dst, Point(poledata.poleLine[0], poledata.poleLine[1]), Point(poledata.poleLine[2], poledata.poleLine[3]), Scalar(0, 0, 255), 3, 8);
	}
	else str_x = "out of window";
	putText(color_dst, str_x, Point(30, 60), FONT_HERSHEY_PLAIN, 1.0, Scalar(0, 255, 0));
	imshow("Detected Lines", color_dst);
}

void saveRGBandDepthMovies(std::string movieName_rgb, std::string movieName_depth) {
	try {
		MyKinectV2 dbg;

		dbg.initializeColor();
		dbg.initializeDepth();
		static cv::VideoWriter rgbWriter(movieName_rgb, cv::VideoWriter::fourcc('X', 'V', 'I', 'D'), 30, cv::Size(dbg.depthWidth, dbg.depthHeight), true);
		static cv::VideoWriter depthWriter(movieName_depth, cv::VideoWriter::fourcc('X', 'V', 'I', 'D'), 30, cv::Size(dbg.depthWidth, dbg.depthHeight), false);

		while (1) {
			dbg.setDepth();
			dbg.setMappedRGB();
			cv::Mat img;
			cv::cvtColor(dbg.RGBImage, img, CV_BGRA2BGR);
			depthWriter << dbg.depthImage;
			rgbWriter << img;
			cv::imshow("depthImage", dbg.depthImage);
			cv::imshow("RGBImage", dbg.RGBImage);

			auto key = cv::waitKey(1);
			if (key == 'q') {
				break;
			}
		}
	}
	catch (std::exception& ex) {
		std::cout << ex.what() << std::endl;
	}
};

//void adjustValues(std::string movieName_rgb, std::string movieName_depth) {
//	try {
//		MyKinectV2 dbg;
//		dbg.initializeDepth();
//		dbg.initializeColor();
//#pragma region initialize_trackbar
//		ValueManager<UINT16> depthManager("values.xml");
//		depthManager.set_value("MINDEPTH", &dbg.MINDEPTH, 8000); depthManager.set_value("MAXDEPTH", &dbg.MAXDEPTH, 8000);
//		depthManager.trackbar("depth");
//
//		ValueManager<int> HSVManager("hsvValues.xml");
//		HSVManager.set_value("min_h", &dbg.hsvKeeper.min_h, 180); HSVManager.set_value("min_s", &dbg.hsvKeeper.min_s, 255); HSVManager.set_value("min_v", &dbg.hsvKeeper.min_v, 255);
//		HSVManager.set_value("max_h", &dbg.hsvKeeper.max_h, 180); HSVManager.set_value("max_s", &dbg.hsvKeeper.max_s, 255); HSVManager.set_value("max_v", &dbg.hsvKeeper.max_v, 255);
//		HSVManager.trackbar("HSV");
//#pragma endregion
//		std::cout << "0: movie mode\n1: camera mode" << std::endl;
//		int flag;
//		std::cin >> flag;
//		std::cout << "s: save values \nq: quit\no: stop" << std::endl;
//		while (1) {
//			switch (flag) {
//			case 0:
//				dbg.setDepthbyMovie(movieName_depth);
//				dbg.setRGBbyMovie(movieName_RGB);
//				break;
//			default:
//				dbg.setDepth();
//				dbg.setMappedRGB();
//			}
//			binarization(dbg.depthImage, dbg.MINDEPTH, dbg.MAXDEPTH);
//
//			dbg.showDistance();
//			cv::imshow("RGBImage", dbg.RGBImage);
//
//			dbg.hsvKeeper.setHSVvalues();
//			dbg.hsvKeeper.setHSVImage(dbg.RGBImage);
//			dbg.hsvKeeper.extractColor();
//			cv::imshow("HSVImage", dbg.hsvKeeper.hsvImage);
//
//			auto key = cv::waitKey(1);
//			if (key == 's') {
//				depthManager.save_value();
//				HSVManager.save_value();
//				std::cout << "saved values!" << std::endl;
//			}
//			else if (key == 'q') break;
//			else if (key == 'o') cv::waitKey(0);
//		}
//	}
//	catch (std::exception& ex) {
//		std::cout << ex.what() << std::endl;
//	}
//}

