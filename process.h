#pragma once
#include"mykinect_v2.h"
#include"xmlManage.h"
void binarization(cv::Mat& image, const int minDepth, const int maxDepth) {
	double mid = 255 * minDepth / 8000.0, mxd = 255 * maxDepth / 8000.0;
	for (int i = 0; i < image.total(); ++i) {
		if (mid < image.data[i] && image.data[i] < mxd) {
			image.data[i] = 255;  //2’l‰»
		}
		else {
			image.data[i] = 0;
		}
	}
}

void saveMovie(std::string movieName_rgb, std::string movieName_depth) {
	try {
		MyKinectV2 dbg;

		dbg.initializeColor();
		dbg.initializeDepth();
		static cv::VideoWriter rgbWriter(movieName_rgb, cv::VideoWriter::fourcc('X', 'V', 'I', 'D'), 30, cv::Size(dbg.depthWidth, dbg.depthHeight), true);
		static cv::VideoWriter depthWriter(movieName_depth, cv::VideoWriter::fourcc('X', 'V', 'I', 'D'), 30, cv::Size(dbg.depthWidth, dbg.depthHeight), false);

		while (1) {
			dbg.setDepth();
			dbg.setMappedRGB();
			cv::Mat img;
			cv::cvtColor(dbg.RGBImage, img, CV_BGRA2BGR);
			depthWriter << dbg.depthImage;
			rgbWriter << img;
			cv::imshow("depthImage", dbg.depthImage);
			cv::imshow("mappedRGB", dbg.RGBImage);

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
		MyKinectV2 dbg;
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
			binarization(dbg.depthImage, dbg.MINDEPTH, dbg.MAXDEPTH);

			dbg.showDistance();
			cv::imshow("RGBImage", dbg.RGBImage);

			dbg.hsvKeeper.setHSVvalues();
			dbg.hsvKeeper.setHSVImage(dbg.RGBImage);
			dbg.hsvKeeper.extractColor();
			cv::imshow("HSVImage", dbg.hsvKeeper.hsvImage);

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

