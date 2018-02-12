#pragma once
#include"mykinect_v2.h"
#include"xmlManage.h"
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

void adjustValues() {
	
}