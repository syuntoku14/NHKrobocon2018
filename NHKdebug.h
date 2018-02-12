#pragma once
#include"mykinect_v2.h"

class KinectDebug :public MyKinectV2 {
public:
#pragma region mouseCallback
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
#pragma endregion

	void showRGB();
	void showDistance();
	void initializeDepth();

	void getHoughLines(cv::Mat& src);
	bool judgeCockThrough();
	bool findShuttle();
};