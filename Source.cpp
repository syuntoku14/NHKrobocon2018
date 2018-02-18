#include"MyKinectV2.h"
#include"image_processing.h"
#include"MySerial.h"
#include"xmlManage.h"
#include<iostream>
#include<numeric>

//ìÆâÊÇÃï€ë∂êÊ
std::time_t now = std::time(nullptr);
std::string movieName_depth = "./shuttleMovies/depth" + std::to_string(now) + ".avi";
std::string movieName_RGB = "./shuttleMovies/RGB" + std::to_string(now) + ".avi";

void test();
void adjustValues(std::string movieName_rgb, std::string movieName_depth);
int main() {
	test();
	//adjustValues("./faultMovies/RGB1.avi", "./faultMovies/depth1.avi");
	return 0;
}

void test() {
	using namespace std;
	vector<PoleData> poledata;
	//savemovie(moviename_rgb,moviename_depth);
	MyKinectV2 kinect;
	kinect.initializeColor();
	kinect.initializeDepth();
	while (1) {
		if (!kinect.setRGBbyMovie("./faultMovies/RGB1.avi")) break;
		cv::imshow("RGB", kinect.RGBImage);
		kinect.setDepthbyMovie("./faultMovies/depth1.avi");
		binarization(kinect.depthImage, 3000, 7000);

		//kinect.hsvKeeper.setHSVImage(kinect.RGBImage);
		//kinect.hsvKeeper.extractColor();
		cv::imshow("depthImage", kinect.depthImage);
		//cv::imshow("hsvImage", kinect.hsvKeeper.hsvImage);
		auto tempdata = setPoleDatabyLSD(kinect.depthImage, 15, 0.90);
		if (tempdata.length > 0) {
			poledata.push_back(tempdata);
			if (poledata.size() > 3) poledata.erase(poledata.begin());
		}
		cout << poledata.size() << endl;
		auto poledata_mean = std::accumulate(poledata.begin(), poledata.end(), PoleData(cv::Vec4f(0.0, 0.0, 0.0, 0.0))) / (float)poledata.size();
		showPoleLine(kinect.depthImage, poledata_mean);

		auto key = cv::waitKey(1);
		if (key == 'q') break;
		else if (key == 's') cv::waitKey(0);
	}
}

void adjustValues(std::string movieName_RGB, std::string movieName_depth) {

	MyKinectV2 kinect;
	kinect.initializeDepth();
	kinect.initializeColor();
#pragma region initialize_trackbar
	ValueManager<UINT16> depthManager("values.xml");
	depthManager.set_value("MINDEPTH", &kinect.MINDEPTH, 8000); depthManager.set_value("MAXDEPTH", &kinect.MAXDEPTH, 8000);
	depthManager.trackbar("depth");

	ValueManager<int> HSVManager("hsvValues.xml");
	HSVManager.set_value("min_h", &kinect.hsvKeeper.min_h, 180); HSVManager.set_value("min_s", &kinect.hsvKeeper.min_s, 255); HSVManager.set_value("min_v", &kinect.hsvKeeper.min_v, 255);
	HSVManager.set_value("max_h", &kinect.hsvKeeper.max_h, 180); HSVManager.set_value("max_s", &kinect.hsvKeeper.max_s, 255); HSVManager.set_value("max_v", &kinect.hsvKeeper.max_v, 255);
	HSVManager.trackbar("HSV");
#pragma endregion

	std::cout << "0: movie mode\n1: camera mode" << std::endl;
	int flag;
	std::cin >> flag;
	std::cout << "o: save values \nq: quit\ns: stop" << std::endl;

	while (1) {
		switch (flag) {
		case 0:
			kinect.setDepthbyMovie(movieName_depth);
			kinect.setRGBbyMovie(movieName_RGB);
			break;
		case 1:
			kinect.setDepth();
			kinect.setMappedRGB();
			break;
		}
		binarization(kinect.depthImage, kinect.MINDEPTH, kinect.MAXDEPTH);

		cv::imshow("depthImage", kinect.depthImage);
		cv::imshow("RGBImage", kinect.RGBImage);

		kinect.hsvKeeper.setHSVvalues();
		kinect.hsvKeeper.setHSVImage(kinect.RGBImage);
		kinect.hsvKeeper.extractColor();
		cv::imshow("HSVImage", kinect.hsvKeeper.hsvImage);

		auto key = cv::waitKey(5);
		if (key == 'o') {
			depthManager.save_value();
			HSVManager.save_value();
			std::cout << "saved values!" << std::endl;
		}
		else if (key == 'q') break;
		else if (key == 's') cv::waitKey(0);
	}
}
