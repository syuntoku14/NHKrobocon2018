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
	float length;
	bool found_flag = false;
	bool found_angle_flag = false;
	int poleDepth;
	char pole_angle;

	cv::Vec4i poledata;
	cv::Vec4f poledata_f; //[1]<[3]
	std::vector<cv::Vec4f> poledata_stack;
	cv::Vec2f poleY; //[0]<[1]
	std::vector<cv::Vec2f> poleY_stack; 
	cv::Vec2i topPosition; //(x,y)
	std::vector<cv::Vec2i> topPosition_stack;
	const int poledata_stack_capa = 3;
	const int poleY_stack_capa = 30;

	cv::Rect ringROI;
	cv::Mat ringImage;
	cv::Point shuttleXY;
	bool success_flag=false;

	void setpoledata(cv::Vec4f line) {
		this->found_flag = true;
		this->poledata_f = line;
		float temp_length = sqrt(pow((line[0] - line[2]), 2) + pow((line[1] - line[3]), 2));

		if (this->length - 30 < temp_length) {
			//poleY_stackにデータをため、y座標の平均値を算出する
			cv::Vec2f temp_poleY;
			temp_poleY[0] = line[1]; temp_poleY[1] = line[3];
			poleY_stack.push_back(temp_poleY);
			if (poleY_stack.size() > poleY_stack_capa) poleY_stack.erase(poleY_stack.begin());
			poleY = std::accumulate(poleY_stack.begin(), poleY_stack.end(), cv::Vec2f::all(0.0)) / (float)poleY_stack.size();

			//poledata_stackにデータを貯めていく
			//これはポールの座標全部蓄えているが、y座標は安定した場合ほとんど普遍なので、その辺を考慮したほうがいいかも
			poledata_stack.push_back(poledata_f);
			if (poledata_stack.size() > poledata_stack_capa) poledata_stack.erase(poledata_stack.begin());
			poledata_f = std::accumulate(poledata_stack.begin(), poledata_stack.end(), cv::Vec4f::all(0.0)) / (float)poledata_stack.size();
			for (int i = 0; i < 4; i++) {
				poledata[i] = (int)poledata_f[i];
			}
			poledata[1] = poleY[0]; poledata[3] = poleY[1]; //poleのy座標を平均値で修正

			this->length = sqrt(pow((poledata[0] - poledata[2]), 2) + pow((poledata[1] - poledata[3]), 2));
			//topPosition
			if (poledata[3] > poledata[1]) { this->topPosition[0] = (int)poledata[0]; this->topPosition[1] = (int)poledata[1]; }
			else { this->topPosition[0] = (int)poledata[2]; this->topPosition[1] = (int)poledata[3]; }
		}
	}

	void setpole_angle() {
		if (found_flag && poleDepth != 0) {
			float x_pxl = ((poledata_f[0] + poledata_f[2]) / 2.0) - 256.0;
			pole_angle = (char)(70.0*(x_pxl / 256.0));
			found_angle_flag = true;
		}
	}

	PoleData() {
		this->found_flag = false;
	}
};


void setPoleDepth(PoleData &poledata, const cv::Mat depthImage, const cv::Mat HSVImage) {
	auto x = (int)((poledata.poledata[0] + poledata.poledata[2]) / 2);
	auto y = (int)((poledata.poledata[1] + poledata.poledata[3]) / 2);
	if (poledata.found_flag) {
		std::vector<float> depth;
		for (int i = -5; i < 6; i++) {
			auto intensity = HSVImage.at<unsigned char>(y, std::max(0, x + i));
			if (intensity > 200) depth.push_back(((float)depthImage.at<unsigned char>(y, std::max(0, x + i))*8000.0 / 255.0));
		}
		depth.size() != 0 ? poledata.poleDepth = std::accumulate(depth.begin(), depth.end(), 0.0) / depth.size() : poledata.poleDepth = -1;
	}
	//std::cout << "poleDepth: " << poledata.poleDepth << std::endl;
};

bool comp(cv::Vec4f& left, cv::Vec4f& right) {
	return left[1] == right[1] ? left[3] > right[3] : left[1] > right[1];
}

auto merge_lines(std::vector<cv::Vec4f>& lines) {
	using namespace std; using namespace cv;

	vector<Vec4f> mergedLines;
	mergedLines.push_back(lines.back()); lines.pop_back();
	while (lines.size() > 0) { //x方向に近い直線を合成
		auto popped_line = lines.back(); lines.pop_back();
		bool merged_flag = false;
		for (auto &merged_line : mergedLines) {
			auto popped_x = (popped_line[0] + popped_line[2]) / 2.0; auto merged_x = (merged_line[0] + merged_line[2]) / 2.0;

			if (abs(popped_x - merged_x) < 15 && (merged_line[3] + 20 > popped_line[1])) {
				auto x0 = (merged_line[0] + popped_line[0]) / 2.0; auto x2 = (merged_line[2] + popped_line[2]) / 2.0;
				merged_line[2] = x2;
				merged_line[3] = max(merged_line[3], popped_line[3]);
				merged_flag = true;
				break;
			}
		}
		if (!merged_flag) mergedLines.push_back(popped_line); //該当しないので追加
	}

	return mergedLines;
}

void setPoleDatabyLSD(cv::Mat &img, PoleData& poledata, int lengthThreshold, double angleThreshold) {
	using namespace cv;
	using namespace std;
	Mat dst;
	int x_min = 0; 
	int x_max = img.cols; 
	int width;
	static int count = 0;
	if (!poledata.found_flag || count % 10 == 0) {
		cv::Canny(img, dst, 50, 200, 3, true);
	}
	else {//探索範囲を狭める
		x_min = max(0, min(poledata.poledata[0], poledata.poledata[2]) - 50);
		x_max = min(img.cols, max(poledata.poledata[0], poledata.poledata[2]) + 50);
		width = x_max - x_min;
		auto rect = cv::Rect(x_min, 0, width, img.rows);
		dst = img(rect);
	}
	count++;
	Ptr<LineSegmentDetector> ls = createLineSegmentDetector(LSD_REFINE_STD);
	vector<Vec4f> lines;
	ls->detect(dst, lines);

	vector<Vec4f> validLines;

	//一定以上の角度,長さのものを抽出
	for (size_t i = 0; i < lines.size(); i++) {
		if (poledata.found_flag) { //範囲を狭めたx座標を元に戻す
			lines[i][0] = lines[i][0] + x_min; lines[i][2] = lines[i][2] + x_min;
		}
		if (lines[i][1] > lines[i][3]) { //[1]<[3]とする
			auto temp_x = lines[i][0]; lines[i][0] = lines[i][2]; lines[i][2] = temp_x;
			auto temp_y = lines[i][1]; lines[i][1] = lines[i][3]; lines[i][3] = temp_y;
		}
		auto angle = atan(abs(lines[i][1] - lines[i][3]) / (abs(lines[i][0] - lines[i][2]) + 1e-10));
		auto length = sqrt(pow((lines[i][0] - lines[i][2]), 2) + pow((lines[i][1] - lines[i][3]), 2));
		if (lines[i][0] < 507 && lines[i][0]>5) { //端っこでバグる
			if (angle > CV_PI / 2.0*angleThreshold && length > lengthThreshold) {
				validLines.push_back(lines[i]);
			}
		}
	}

	if (validLines.size() > 0) {
		sort(validLines.begin(), validLines.end(), comp); //yが小さいものが後ろ
		auto mergedLines = merge_lines(validLines);
		//// Show found lines
		//Mat drawnLines(dst);
		//ls->drawSegments(drawnLines, mergedLines);
		//imshow("Standard refinement", drawnLines);

		auto max_length = 0.0;
		Vec4f longest_line = mergedLines[0];
		for (auto &merged_line : mergedLines) {
			auto length_merged = sqrt(pow((merged_line[0] - merged_line[2]), 2) + pow((merged_line[1] - merged_line[3]), 2));
			if (max_length < length_merged) {
				max_length = length_merged; longest_line = merged_line;
			}
		}
		if (poledata.length - 30 < max_length) {
			poledata.setpoledata(longest_line);
		}
	}
};

void showPoleLine(cv::Mat &img, PoleData poledata) {
	using namespace std;
	using namespace cv;
	Mat color_dst;
	string str_x;
	cvtColor(img, color_dst, CV_GRAY2BGR);
	if (poledata.found_flag) {
		str_x = " x coordinates= " + to_string(poledata.poledata[0]) + " pole Length: " + to_string(poledata.length);
		line(color_dst, Point(poledata.poledata[0], poledata.poledata[1]), Point(poledata.poledata[2], poledata.poledata[3]), Scalar(0, 0, 255), 3, 8);
	}
	else str_x = "out of window";
	putText(color_dst, str_x, Point(30, 60), FONT_HERSHEY_PLAIN, 1.0, Scalar(0, 255, 0));
	imshow("Detected Lines", color_dst);
}

void showPoleLine_when_Kinect(cv::Mat &img, PoleData poledata) {
	using namespace std;
	using namespace cv;
	Mat color_dst;
	string str_x;
	cvtColor(img, color_dst, CV_GRAY2BGR);
	if (poledata.found_flag) {
		str_x = " x coordinates= " + to_string(poledata.poledata[0]) + " pole Length: " + to_string(poledata.length);
		line(color_dst, Point(poledata.poledata[0], poledata.poledata[1]), Point(poledata.poledata[2], poledata.poledata[3]), Scalar(0, 0, 255), 3, 8);
	}
	else str_x = "out of window";
	putText(color_dst, str_x, Point(30, 60), FONT_HERSHEY_PLAIN, 1.0, Scalar(0, 255, 0));
	resize(color_dst, color_dst, Size(), 0.4, 0.4);
	imshow("Detected Lines", color_dst);
}
//auto findShuttleCoordinate_byTracking(cv::Mat HSVImage, ShuttleData &shuttledata) {
	//	using namespace cv;
	//	using namespace std;
	//	Mat dst = HSVImage;
	//	Mat element = Mat::ones(3, 3, CV_8UC1);
	//	dilate(dst, dst, element, Point(-1, -1), 3); //膨張処理3回
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

void find_shuttleLoc(PoleData& poledata, cv::Mat& img) {
	using namespace std;
	cv::Mat color_ring;

	float ringRad, trueLength;
	poledata.ringtype == 'g' ? (ringRad = 400.0, trueLength = 3000.0) : (ringRad = 400.0, trueLength = 2000.0);
	const int KERNELSIZE = 3;
	const int FILTERTH = 150;
	static cv::Mat kernel = cv::Mat::ones(KERNELSIZE, KERNELSIZE, CV_64F) / (double)(KERNELSIZE*KERNELSIZE);
	cv::Scalar sum_of_ring;
	if (poledata.found_flag) {
		int sideLength = max((int)(poledata.length*ringRad / trueLength)-10,0);
		int x = std::max(poledata.topPosition[0] - sideLength, 0);
		int y = std::max(poledata.topPosition[1] - sideLength * 2, 0);
		poledata.ringROI = cv::Rect(x, y, sideLength * 2, sideLength * 2);
		poledata.ringImage = img(poledata.ringROI);
		//cv::imshow("ringImage", poledata.ringImage);
		cv::filter2D(poledata.ringImage, poledata.ringImage, -1, kernel);
		//cv::threshold(poledata.ringImage, poledata.ringImage, FILTERTH, 0, cv::THRESH_TOZERO); //多分いらない　convedimageで十分
		sum_of_ring = cv::sum(poledata.ringImage);
		poledata.success_flag = false;
		cout << "sum of ring" << (int)sum_of_ring[0] << endl;
		if (sum_of_ring[0] > 10) { 
			cv::minMaxLoc(poledata.ringImage, NULL, NULL, NULL, &poledata.shuttleXY);
			cout << "shutlleXY: " << poledata.shuttleXY << endl;
			poledata.success_flag = true;
		}
		cv::imshow("ringImage", poledata.ringImage);
	}
}