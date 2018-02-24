#pragma once
#include"MyKinectV2.h"
#include"xmlManage.h"
#include<numeric>
#include"image_processing.h"

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

class PoleData {
public:
	char ringtype;
	int length;
	cv::Vec4i poleLine;
	cv::Vec4f poleLine_f;
	cv::Vec2i topPosition;
	int poleDepth;
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
	PoleData() {
		this->poleDepth = -1;
		this->length = -1;
	};
};

void setPoleDepthbyKinect(PoleData &poledata, const std::vector<UINT16> &depthBuffer, const cv::Mat HSVImage) {
	using namespace std;
	auto x = (int)((poledata.poleLine[0] + poledata.poleLine[2]) / 2);
	auto y = (int)((poledata.poleLine[1] + poledata.poleLine[3]) / 2);
	auto cols = HSVImage.cols;
	if (poledata.length > 0) {
		for (int i = -5; i < 6; i++) {
			auto coord = cols * y + max(0, x + i);
			if (HSVImage.data[coord] > 200) {
				poledata.poleDepth = depthBuffer[coord];
				break;
			}
		}
	}
	std::cout << "poleDepth: " << poledata.poleDepth << endl;
};

void setPoleDepthbyMovie(PoleData &poledata, const cv::Mat depthImage, const cv::Mat HSVImage) {
	auto x = (int)((poledata.poleLine[0] + poledata.poleLine[2]) / 2);
	auto y = (int)((poledata.poleLine[1] + poledata.poleLine[3]) / 2);
	if (poledata.length > 0) {
		std::vector<float> depth;
		for (int i = -5; i < 6; i++) {
			auto intensity = HSVImage.at<unsigned char>(y, std::max(0, x + i));
			if (intensity > 200) depth.push_back(((float)depthImage.at<unsigned char>(y, std::max(0, x + i))*8000.0 / 255.0));
		}
		depth.size() != 0 ? poledata.poleDepth = std::accumulate(depth.begin(), depth.end(), 0.0) / depth.size() : poledata.poleDepth = -1;
	}
	std::cout << "poleDepth: " << poledata.poleDepth << std::endl;
}

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

auto setPoleDatabyLSD(cv::Mat &img, int lengthThreshold, double angleThreshold) {
	using namespace cv;
	using namespace std;
	Mat dst;
	cv::Canny(img, dst, 100, 500, 3, true);
	Ptr<LineSegmentDetector> ls = createLineSegmentDetector(LSD_REFINE_STD);
	vector<Vec4f> lines;
	ls->detect(dst, lines);

	//(y,x)ÇÃèá
	vector<pair<float, float> >endPoints;

	//àÍíËà»è„ÇÃäpìx,í∑Ç≥ÇÃÇ‡ÇÃÇíäèo
	for (size_t i = 0; i < lines.size(); i++) {
		double angle = atan(abs(lines[i][1] - lines[i][3]) / (abs(lines[i][0] - lines[i][2]) + 1e-10));
		int length = (int)sqrt(pow((lines[i][0] - lines[i][2]), 2) + pow((lines[i][1] - lines[i][3]), 2));
		if (lines[i][0] < 507 && lines[i][0]>5) { //í[Ç¡Ç±Ç≈ÉoÉOÇÈ
			if (angle > CV_PI / 2.0*angleThreshold && length > lengthThreshold) {
				endPoints.push_back(make_pair(lines[i][1], lines[i][0]));
				endPoints.push_back(make_pair(lines[i][3], lines[i][2]));
			}
		}
	}

	if (lines.size()) {
		sort(endPoints.begin(), endPoints.end());
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

//auto findShuttleCoordinate_byTracking(cv::Mat HSVImage, ShuttleData &shuttledata) {
	//	using namespace cv;
	//	using namespace std;
	//	Mat dst = HSVImage;
	//	Mat element = Mat::ones(3, 3, CV_8UC1);
	//	dilate(dst, dst, element, Point(-1, -1), 3); //ñcí£èàóù3âÒ
	//	vector<vector<Point> > contours;
	//	findContours(dst, contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE);
	//	double max_area = 0;
	//	int max_area_contour = -1;
	//	for (int j = 0; j < contours.size(); j++) {
	//		double area = contourArea(contours.at(j));
	//		if (max_area < area) {
	//			max_area = area;
	//			max_area_contour = j;
	//		}
	//
	//		int count = contours.at(max_area_contour).size();
	//		double x = 0;
	//		double y = 0;
	//		for (int k = 0; k < count; k++) {
	//			x += contours.at(max_area_contour).at(k).x;
	//			y += contours.at(max_area_contour).at(k).y;
	//		}
	//		x /= count;
	//		y /= count;
	//
	//		//circle(dst, Point(x, y), 50, Scalar(0, 0, 255), 3, 4);
	//		//imshow("centor",dst);
	//	}
	//}