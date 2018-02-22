#include"MyKinectV2.h"
#include"image_processing.h"
#include"MySerial.h"
#include"xmlManage.h"
#include<iostream>
#include<numeric>

//動画の保存先
std::time_t now = std::time(nullptr);
std::string movieName_depth = "./shuttleMovies/depth" + std::to_string(now) + ".avi";
std::string movieName_RGB = "./shuttleMovies/RGB" + std::to_string(now) + ".avi";

void test();
void test2();
void adjustValues(std::string movieName_rgb, std::string movieName_depth);

//基本的に取れていないデータには-1が入る
int main() {
	//test();
	adjustValues("./faultMovies/RGB1.avi", "./faultMovies/depth1.avi");
	return 0;
}

void test2() {
	MyKinectV2 kinect;
	kinect.initializeColor();
	kinect.initializeDepth();
	while (1) {
		if (!kinect.setRGBbyMovie("./faultMovies/RGB1.avi")) break;
		cv::imshow("RGB", kinect.RGBImage);
		kinect.setDepthbyMovie("./faultMovies/depth1.avi");
		binarization(kinect.depthImage, 3000, 7000);
		kinect.hsvKeeper.setHSVImage(kinect.RGBImage);
		kinect.hsvKeeper.extractColor();
		cv::imshow("depthImage", kinect.depthImage);
		cv::imshow("hsvImage", kinect.hsvKeeper.hsvImage);
		convBinarizaionByHsv(kinect.hsvKeeper.hsvImage, kinect.depthImage); //depthImageを変換
		cv::imshow("convedImage", kinect.depthImage);

		auto key = cv::waitKey(1);
		if (key == 'q') break;
		else if (key == 's') cv::waitKey(0);
	}
}

void test() {
	using namespace std;
	vector<PoleData> poledatas;
	//savemovie(moviename_rgb,moviename_depth);
	MyKinectV2 kinect;
	kinect.initializeColor();
	kinect.initializeDepth();
	cv::Mat poleImage;
	while (1) {
		if (!kinect.setRGBbyMovie("./faultMovies/RGB1.avi")) break;
		kinect.setDepthbyMovie("./faultMovies/depth1.avi");
		cv::imshow("RGB", kinect.RGBImage);
		kinect.hsvKeeper.setHSVvalues("hsvValues_red.xml");
		kinect.hsvKeeper.setHSVImage(kinect.RGBImage);
		kinect.hsvKeeper.extractColor();
		//cv::imshow("HSVImage", kinect.hsvKeeper.hsvImage);
		cv::imshow("depthImage", kinect.depthImage);
		poleImage=convBinarizaionByHsv(kinect.hsvKeeper.hsvImage, kinect.depthImage); //poleImageを取得

		auto tempdata = setPoleDatabyLSD(poleImage, 0, 0.90);
		//poledataの平均を算出
		if (tempdata.length > 0) {
			poledatas.push_back(tempdata);
			if (poledatas.size() > 3) poledatas.erase(poledatas.begin());
		}
		auto poledata = std::accumulate(poledatas.begin(), poledatas.end(), PoleData(cv::Vec4f(0.0, 0.0, 0.0, 0.0))) / (float)poledatas.size();
		//setPoleDepthbyKinect(poledata, kinect.depthBuffer, kinect.hsvKeeper.hsvImage);
		setPoleDepthbyMovie(poledata,kinect.depthImage,kinect.hsvKeeper.hsvImage);
		showPoleLine(kinect.depthImage, poledata);

		auto key = cv::waitKey(1);
		if (key == 'q') break;
		else if (key == 's') cv::waitKey(0);
	}
}

void adjustValues(std::string movieName_RGB, std::string movieName_depth) {

	std::cout << "0: movie mode\n1: camera mode" << std::endl;
	int flag;
	std::cin >> flag;
	std::cout << "chose color to save\nr: red b: blue y: yellow" << std::endl;
	char color_flag;
	std::cin >> color_flag;
	std::string hsvfile_name;
	switch(color_flag) {
	case 'r':
		hsvfile_name = "hsvValues_red.xml"; 
		break;
	case 'b':
		hsvfile_name = "hsvValues_blue.xml";
		break;
	case 'y':
		hsvfile_name = "hsvValues_yellow.xml";
		break;
	}
	std::cout << "o: save values \nq: quit\ns: stop" << std::endl;

	MyKinectV2 kinect;
	kinect.initializeDepth();
	kinect.initializeColor();
#pragma region initialize_trackbar
	ValueManager<int> HSVManager(hsvfile_name);
	HSVManager.set_value("min_h", &kinect.hsvKeeper.min_h, 180); HSVManager.set_value("min_s", &kinect.hsvKeeper.min_s, 255); HSVManager.set_value("min_v", &kinect.hsvKeeper.min_v, 255);
	HSVManager.set_value("max_h", &kinect.hsvKeeper.max_h, 180); HSVManager.set_value("max_s", &kinect.hsvKeeper.max_s, 255); HSVManager.set_value("max_v", &kinect.hsvKeeper.max_v, 255);
	HSVManager.trackbar("HSV");
#pragma endregion
	std::cout << hsvfile_name << std::endl;
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

		kinect.hsvKeeper.setHSVvalues(hsvfile_name);
		kinect.hsvKeeper.setHSVImage(kinect.RGBImage);
		kinect.hsvKeeper.extractColor();
		cv::imshow("HSVImage", kinect.hsvKeeper.hsvImage);

		auto key = cv::waitKey(100);
		if (key == 'o') {
			HSVManager.save_value();
			std::cout << "saved values!" << std::endl;
		}
		else if (key == 'q') break;
		else if (key == 's') cv::waitKey(0);
	}
}
