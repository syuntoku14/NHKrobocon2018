#pragma once
#include"mykinect_v2.h"

class KinectDebug :public MyKinectV2 {
public:
#pragma region mouseCallback
	static void mouseCallback(int event, int x, int y, int flags, void* userdata) {
		// �����ɓn����this�|�C���^���o�R���ă����o�֐��ɓn��
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