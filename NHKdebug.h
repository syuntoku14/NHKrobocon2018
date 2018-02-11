#pragma once
#include"mykinect_v2.h"

class KinectDebug :public MyKinectV2 {
public:
	static void mouseCallback(int event, int x, int y, int flags, void* userdata) {
		// 引数に渡したthisポインタを経由してメンバ関数に渡す
		auto pThis = (KinectDebug*)userdata;
		pThis->mouseCallback(event, x, y, flags);
	};
	void mouseCallback(int event, int x, int y, int flags) {
		if (event == CV_EVENT_LBUTTONDOWN) {
			depthPointX = x;
			depthPointY = y;
		}
	};
	void saveRGBMovie(std::string movieName);
	void saveDepthMovie(std::string movieName);

	void useMovie(std::string movieName_rgb,std::string movieName_depth);
	
	void showRGB();
	void showDistance();
	void initializeDepth();

	void getHoughLines(cv::Mat& src);
	bool judgeCockThrough();
	bool findShuttle();
};