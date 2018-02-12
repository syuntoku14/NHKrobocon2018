#include"mykinect_v2.h"
#include"NHKdebug.h"
#include"MySerial.h"
#include"xmlManage.h"
#include<iostream>

//動画の保存先と利用先
std::time_t now = std::time(nullptr);
std::string movieName_depth = "./shuttleMovies/depth" + std::to_string(now) + ".avi";
std::string movieName_RGB = "./shuttleMovies/RGB" + std::to_string(now) + ".avi";

const int COMPORT = 5;
char ringtemp[1];

void robocon();
void saveMovie(std::string movieName_rgb, std::string movieName_depth);
void useMovie(std::string movieName_rgb, std::string movieName_depth);
void adjustValues(std::string movieName_rgb, std::string movieName_depth);
void test();
//MySerial serial(COMPORT);

int main()
{
	ringtemp[0] = 'r';
	//信号が送られるまで待機
	//serial.recieveData(ringtemp);

	test();
	//robocon();
	//adjustValues("./successMovies/RGB1.avi","./successMovies/depth1.avi");
	//saveMovie();
	//useMovie("./successMovies/RGB1.avi", "./successMovies/depth1.avi");
	return 0;
}

void robocon() {
	try {
		MyKinectV2 app;
		//ポールの種類を格納
		app.poleLine.ringtype = ringtemp[0];
		app.initializeDepth();
		while (1) {
			app.getShuttleLoc();
			//serial.sendData(app.poleX);
			auto key = cv::waitKey(1);
			if (key == 'q') {
				break;
			}
		}
	}
	catch (std::exception& ex) {
		std::cout << ex.what() << std::endl;
	}
}

void adjustValues(std::string movieName_rgb, std::string movieName_depth) {
	try {
		KinectDebug dbg;
		dbg.poleLine.ringtype = ringtemp[0];
		dbg.initializeDepth();
		dbg.initializeColor();
#pragma region initialize_trackbar
		ValueManager<UINT16> depthManager("values.xml");
		depthManager.set_value("MINDEPTH", &dbg.MINDEPTH, 8000); depthManager.set_value("MAXDEPTH", &dbg.MAXDEPTH, 8000);
		depthManager.trackbar("depth");

		ValueManager<int> HSVManager("hsvValues.xml");
		HSVManager.set_value("min_h", &dbg.hsvKeeper.min_h, 180); HSVManager.set_value("min_s", &dbg.hsvKeeper.min_s, 255); HSVManager.set_value("min_v", &dbg.hsvKeeper.min_v, 255);
		HSVManager.set_value("max_h", &dbg.hsvKeeper.max_h, 180); HSVManager.set_value("max_s", &dbg.hsvKeeper.max_s, 255); HSVManager.set_value("max_v", &dbg.hsvKeeper.max_v, 255);
		HSVManager.trackbar("HSV");
#pragma endregion
		std::cout << "0: movie mode\n1: camera mode" << std::endl;
		int flag;
		std::cin >> flag;
		std::cout << "s: save values \nq: quit\no: stop" << std::endl;
		static cv::VideoCapture cap_rgb(movieName_rgb), cap_depth(movieName_depth);
		cv::Mat temp_rgb, temp_depth;
		while (1) {
			switch (flag) {
			case 0:
				cap_rgb >> temp_rgb; cap_depth >> temp_depth;
				cv::cvtColor(temp_rgb, dbg.RGBImage, CV_BGR2BGRA); cv::cvtColor(temp_depth, dbg.depthImage, CV_RGB2GRAY);
				break;
			case 1:
				dbg.setDepth();
				dbg.setMappedRGB();
			}
			dbg.binarization(dbg.depthImage,dbg.MINDEPTH,dbg.MAXDEPTH);
			
			dbg.showDistance();
			dbg.showRGB();

			dbg.hsvKeeper.setHSVvalues();
			dbg.hsvKeeper.setHSVImage(dbg.RGBImage);
			dbg.hsvKeeper.extractColor();
			dbg.hsvKeeper.showHSV();

			auto key = cv::waitKey(1);
			if (key == 's') {
				depthManager.save_value();
				HSVManager.save_value();
				std::cout << "saved values!" << std::endl;
			}
			else if (key == 'q') break;
			else if (key == 'o') cv::waitKey(0);
		}
	}
	catch (std::exception& ex) {
		std::cout << ex.what() << std::endl;
	}
}

void saveMovie(std::string movieName_rgb, std::string movieName_depth) {
	try {
		KinectDebug dbg;
		//ポールの種類を格納
		dbg.poleLine.ringtype = ringtemp[0];

		dbg.initializeColor();
		static cv::VideoWriter rgbWriter(movieName_rgb, cv::VideoWriter::fourcc('X', 'V', 'I', 'D'), 30, cv::Size(dbg.colorWidth, dbg.colorHeight), true);
		dbg.initializeDepth();
		static cv::VideoWriter depthWriter(movieName_depth, cv::VideoWriter::fourcc('X', 'V', 'I', 'D'), 30, cv::Size(dbg.depthWidth, dbg.depthHeight), false);

		while (1) {
			dbg.setDepth();
			dbg.setRGB();
			depthWriter << dbg.depthImage;
			cv::cvtColor(dbg.RGBImage, dbg.RGBImage,CV_BGRA2BGR);
			rgbWriter << dbg.RGBImage;
			cv::imshow("depthImage", dbg.depthImage);
			dbg.showRGB();

			auto key = cv::waitKey(1);
			if (key == 'q') {
				break;
			}
		}
	}
	catch (std::exception& ex) {
		std::cout << ex.what() << std::endl;
	}
}

void useMovie(std::string movieName_rgb, std::string movieName_depth) {
	try {
		KinectDebug dbg;
		dbg.initializeDepth();
		dbg.initializeColor();
		dbg.poleLine.ringtype = ringtemp[0];
		static cv::VideoCapture cap_rgb(movieName_rgb), cap_depth(movieName_depth);
		cv::Mat temp_rgb, temp_depth;


		while (1) {
			cap_rgb >> temp_rgb; cap_depth >> temp_depth;
			if (temp_rgb.empty() || temp_depth.empty()) break;
			auto key = cv::waitKey(1);
			if (key == 'q') break;
			else if (key == 's') cv::waitKey(0);
			cv::cvtColor(temp_depth, dbg.depthImage, CV_RGB2GRAY);
			dbg.binarization(dbg.depthImage, dbg.MINDEPTH, dbg.MAXDEPTH);
			cv::medianBlur(dbg.depthImage, dbg.depthImage, 5);
			dbg.showDistance();
			dbg.getHoughLines(dbg.depthImage);

			cvtColor(temp_rgb, dbg.RGBImage, CV_BGR2BGRA);
			dbg.hsvKeeper.setHSVImage(dbg.RGBImage);
			dbg.hsvKeeper.extractColor();
			dbg.binarization(dbg.hsvKeeper.hsvImage, dbg.MINDEPTH, dbg.MAXDEPTH);
			cv::medianBlur(dbg.hsvKeeper.hsvImage, dbg.hsvKeeper.hsvImage, 5);
			dbg.hsvKeeper.showHSV();
			dbg.getHoughLines(dbg.hsvKeeper.hsvImage);
		}
	}	
	catch (std::exception& ex) {
		std::cout << ex.what() << std::endl;
	}
}

void test() {
	KinectDebug dbg;
	dbg.poleLine.ringtype = ringtemp[0];
	dbg.initializeDepth();
	dbg.initializeColor();
	while (1) {
		dbg.setDepth();
		dbg.setMappedRGB();
		dbg.showDistance();
		cv::imshow("test",dbg.RGBImage);
		auto key = cv::waitKey(1);
		if (key == 'q') {
			break;
		}
	}
}