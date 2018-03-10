#include"MyKinectV2.h"
#include"image_processing.h"
#include"MySerial.h"
#include"xmlManage.h"
#include<iostream>
#include<numeric>
#include"PoleAndShuttlePrediction.h"
#include"MySerial.h"

#define COMPORT 4 

//動画の保存先
std::time_t now = std::time(nullptr);
std::string movieName_depth = "./shuttleMovies/depth" + std::to_string(now) + ".avi";
std::string movieName_RGB = "./shuttleMovies/RGB" + std::to_string(now) + ".avi";

void test();
void LSDtestByMovie(char ringtype);
void LSDtestByKinect(char ringtype);
void adjustValues(std::string movieName_rgb, std::string movieName_depth);

//基本的に取れていないデータには-1が入る

MySerial serial(COMPORT, false);

int main() {
	using namespace std;
	char ringtype[1];
	serial.recieveData(ringtype);
	cout << ringtype[0] << endl;
	serial.sendData('c');

	//saveRGBandDepthMovies(movieName_RGB,movieName_depth);
	//test();
	//LSDtestByKinect(ringtype[0]);
	LSDtestByMovie(ringtype[0]);
	//adjustValues("./faultMovies/RGByellow.avi", "./faultMovies/depthyellow.avi");
	return 0;
}

void test() {
	using namespace cv;
	MyKinectV2 kinect;
	kinect.initializeColor();
	kinect.initializeDepth();
	kinect.initializeMulti();
	Mat rgb, depth;
	while (1) {
		//kinect.setDepthandMappedRGB();
		kinect.setMappedDepthandRGB();
		resize(kinect.RGBImage, rgb, cv::Size(), 0.4, 0.4);
		resize(kinect.depthImage, depth, cv::Size(), 0.4, 0.4);
		imshow("rgb", rgb);
		imshow("depth", depth);
		auto key = cv::waitKey(1);
		if (key == 'q') break;
		else if (key == 's') cv::waitKey(0);
	}
}

void LSDtestByMovie(char ringtype) {
	using namespace std;
	PoleData poledata;
	poledata.ringtype = ringtype;

	MyKinectV2 kinect;
	kinect.initializeColor();
	kinect.initializeDepth();
	cv::Mat convedImage;
	kinect.hsvKeeper.initHSVvalues("hsvValues_red.xml");
	while (1) {
		if (!kinect.setRGBbyMovie("./faultMovies/RGB1.avi")) break;
		kinect.setDepthbyMovie("./faultMovies/depth1.avi");
		cv::imshow("RGB", kinect.RGBImage);
		kinect.hsvKeeper.setHSVvalues();
		kinect.hsvKeeper.setHSVImage(kinect.RGBImage);
		kinect.hsvKeeper.extractColor();
		cv::imshow("HSVImage", kinect.hsvKeeper.hsvImage);
		cv::imshow("depthImage", kinect.depthImage);

		//画像処理パート
		convedImage = convBinarizaionByHsv(kinect.hsvKeeper.hsvImage, kinect.depthImage); //convedImageを取得

		setPoleDatabyLSD(convedImage, poledata, 0, 0.90);
		setPoleDepthbyMovie(poledata, kinect.depthImage, kinect.hsvKeeper.hsvImage);
		showPoleLine(kinect.depthImage, poledata);
		find_shuttleLoc(poledata, kinect.depthImage);

		poledata.setpole_angle();
		if (poledata.found_angle_flag) {
			cout << (int)poledata.pole_angle << endl;
			serial.sendData(poledata.pole_angle);
		}
		auto key = cv::waitKey(1);
		if (key == 'q') break;
		else if (key == 's') cv::waitKey(0);
	}
}

void LSDtestByKinect(char ringtype) {

	using namespace std;
	PoleData poledata;
	poledata.ringtype = ringtype;

	MyKinectV2 kinect;
	kinect.initializeColor();
	kinect.initializeDepth();
	kinect.initializeMulti();
	cv::Mat convedImage;
	kinect.hsvKeeper.initHSVvalues("hsvValues_red.xml");
	cv::Mat rgb, depth;
	while (1) {
		kinect.setMappedDepthandRGB();
		cv::resize(kinect.RGBImage, rgb, cv::Size(), 0.4, 0.4);
		cv::imshow("RGB", rgb);
		kinect.hsvKeeper.setHSVvalues();
		kinect.hsvKeeper.setHSVImage(kinect.RGBImage);
		kinect.hsvKeeper.extractColor();
		cv::resize(kinect.depthImage, depth, cv::Size(), 0.4, 0.4);
		cv::imshow("depthImage", depth);
		convedImage = convBinarizaionByHsv(kinect.hsvKeeper.hsvImage, kinect.depthImage); //convedImageを取得

		setPoleDatabyLSD(convedImage, poledata, 0, 0.90);
		setPoleDepthbyMovie(poledata, kinect.depthImage, kinect.hsvKeeper.hsvImage);
		showPoleLine_when_Kinect(kinect.depthImage, poledata);
		find_shuttleLoc(poledata, kinect.depthImage);
		poledata.setpole_angle();
		if (poledata.found_angle_flag) {
			cout << (int)poledata.pole_angle << endl;
			serial.sendData(poledata.pole_angle);
		}
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
	switch (color_flag) {
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

	kinect.hsvKeeper.initHSVvalues(hsvfile_name);
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

		auto key = cv::waitKey(100);
		if (key == 'o') {
			HSVManager.save_value();
			std::cout << "saved values!" << std::endl;
		}
		else if (key == 'q') break;
		else if (key == 's') cv::waitKey(0);
	}
}
