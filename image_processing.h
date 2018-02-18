#pragma once
#include"MyKinectV2.h"
#include"xmlManage.h"

void binarization(cv::Mat& image, const int minDepth, const int maxDepth) {
	double mid = 255 * minDepth / 8000.0, mxd = 255 * maxDepth / 8000.0;
	for (int i = 0; i < image.total(); ++i) {
		if (mid < image.data[i] && image.data[i] < mxd) {
			image.data[i] = 255;  //2値化
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
	PoleData(cv::Vec4i line) {
		this->length = (int)sqrt(pow((line[0] - line[2]), 2) + pow((line[1] - line[3]), 2));
		this->poleLine = line;
	};
	PoleData(cv::Vec4f line) {
		this->length = (int)sqrt(pow((line[0] - line[2]), 2) + pow((line[1] - line[3]), 2));
		this->poleLine_f = line;
		for (int i = 0; i < 4; i++) {
			poleLine[i] = (int)poleLine_f[i];
		}
	};
	PoleData operator +(const PoleData& other) const {
		return { poleLine_f + other.poleLine_f };
	};
	PoleData operator / (const float num) const {
		return { poleLine_f / num };
	};
	PoleData() { this->length = -1; };
};

class HoughLineParamaters {
public:
	int rho, threshold, minLineLength, maxLineGap;
	double theta, angleThreshold;
	HoughLineParamaters(int rho, double theta, int threshold, int minLineLength, int maxLineGap, double angleThreshold) {
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

	//一定以上の角度のものを抽出
	for (size_t i = 0; i < lines.size(); i++) {
		double angle = atan(abs(lines[i][1] - lines[i][3]) / (abs(lines[i][0] - lines[i][2]) + 1e-10));
		if (lines[i][0] < 507 && lines[i][0]>5) { //端っこでバグる
			if (angle > CV_PI / 2.0*params.angleThreshold) {
				int length = (int)sqrt(pow((lines[i][0] - lines[i][2]), 2) + pow((lines[i][1] - lines[i][3]), 2));
				poles.push(PoleData(length, lines[i]));
			}
		}
	}

	if (!poles.empty()) {
		//長さが最大のlineをpoledataに格納
		poledata = poles.top();
		//頂点情報を格納
		if (poledata.poleLine[1] >= poledata.poleLine[3]) { poledata.topPosition[0] = poledata.poleLine[2]; poledata.topPosition[1] += poledata.poleLine[3]; }
		else { poledata.topPosition[0] = poledata.poleLine[0]; poledata.topPosition[1] += poledata.poleLine[1]; }
	}

};

auto setPoleDatabyLSD(cv::Mat &img, int lengthThreshold, double angleThreshold) {
	using namespace cv;
	using namespace std;
	Mat dst;
	medianBlur(img, dst, 7);
	Mat kernel = (Mat_<float>(3, 3) << 1, 0, -1, 1, 0, -1, 1, 0, -1);
	filter2D(dst, dst, -1, kernel);
	Canny(dst, dst, 100, 500, 3, true);

	imshow("canny", dst);
	Ptr<LineSegmentDetector> ls = createLineSegmentDetector(LSD_REFINE_STD);
	vector<Vec4f> lines;
	ls->detect(dst, lines);

	vector<pair<float, float> >endPoints; //(y,x)の順

	//一定以上の角度,長さのものを抽出
	for (size_t i = 0; i < lines.size(); i++) {
		double angle = atan(abs(lines[i][1] - lines[i][3]) / (abs(lines[i][0] - lines[i][2]) + 1e-10));
		int length = (int)sqrt(pow((lines[i][0] - lines[i][2]), 2) + pow((lines[i][1] - lines[i][3]), 2));
		if (lines[i][0] < 507 && lines[i][0]>5) { //端っこでバグる
			if (angle > CV_PI / 2.0*angleThreshold && length > lengthThreshold) {
				endPoints.push_back(make_pair(lines[i][1], lines[i][0]));
				endPoints.push_back(make_pair(lines[i][3], lines[i][2]));
			}
		}
	}

	if (lines.size()) {
		sort(endPoints.begin(), endPoints.end());
		for (auto i = endPoints.begin(); i < endPoints.end(); i++) {
			cout << i->first << endl;
		}
		cout << endl;
		auto topPos = *endPoints.begin();
		auto btmPos = *(endPoints.end() - 1);
		return PoleData(Vec4f(topPos.second, topPos.first, btmPos.second, btmPos.first));
	}

	return PoleData();
};

void showPoleLine(cv::Mat &img, PoleData poledata) {
	using namespace std;
	using namespace cv;
	Mat color_dst;
	string str_x;
	cvtColor(img, color_dst, CV_GRAY2BGR);
	if (poledata.length > 0) {
		str_x = " x coordinates= " + to_string(poledata.poleLine[0]) + " pole Length: " + to_string(poledata.length);
		line(color_dst, Point(poledata.poleLine[0], poledata.poleLine[1]), Point(poledata.poleLine[2], poledata.poleLine[3]), Scalar(0, 0, 255), 3, 8);
	}
	else str_x = "out of window";
	putText(color_dst, str_x, Point(30, 60), FONT_HERSHEY_PLAIN, 1.0, Scalar(0, 255, 0));
	imshow("Detected Lines", color_dst);
}

void saveRGBandDepthMovies(std::string movieName_rgb, std::string movieName_depth) {
	try {
		MyKinectV2 kinect;

		kinect.initializeColor();
		kinect.initializeDepth();
		static cv::VideoWriter rgbWriter(movieName_rgb, cv::VideoWriter::fourcc('X', 'V', 'I', 'D'), 30, cv::Size(kinect.depthWidth, kinect.depthHeight), true);
		static cv::VideoWriter depthWriter(movieName_depth, cv::VideoWriter::fourcc('X', 'V', 'I', 'D'), 30, cv::Size(kinect.depthWidth, kinect.depthHeight), false);

		while (1) {
			kinect.setDepth();
			kinect.setMappedRGB();
			cv::Mat img;
			cv::cvtColor(kinect.RGBImage, img, CV_BGRA2BGR);
			depthWriter << kinect.depthImage;
			rgbWriter << img;
			cv::imshow("depthImage", kinect.depthImage);
			cv::imshow("RGBImage", kinect.RGBImage);

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

