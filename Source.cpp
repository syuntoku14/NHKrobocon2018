#include"MyKinectV2.h"
#include"image_processing.h"
#include"MySerial.h"
#include"xmlManage.h"
#include<iostream>

//ìÆâÊÇÃï€ë∂êÊ
std::time_t now = std::time(nullptr);
std::string movieName_depth = "./shuttleMovies/depth" + std::to_string(now) + ".avi";
std::string movieName_RGB = "./shuttleMovies/RGB" + std::to_string(now) + ".avi";

void test();

int main(){
	test();
	return 0;
}

void test() {
	HoughLineParamaters params(2, CV_PI/360.0, 50, 150, 10, 0.0);
	PoleData poledata;
	//savemovie(moviename_rgb,moviename_depth);
	MyKinectV2 kinect;
	kinect.initializeColor();
	kinect.initializeDepth();
	while (1) {
		if (!kinect.setRGBbyMovie("./faultMovies/RGB1.avi")) break;
		cv::imshow("RGB", kinect.RGBImage);
		kinect.setDepthbyMovie("./faultMovies/depth1.avi");
		cv::imshow("depthImage", kinect.depthImage);
		setPoleDatabyLSD(kinect.depthImage, poledata, 0);
		//setPoleDatabyHoughLine(kinect.depthImage, poledata, params);
		showPoleLine(kinect.depthImage, poledata);
		std::cout << "pole Length:" << poledata.length << std::endl;

		auto key = cv::waitKey(1);
		if (key == 'q') break;
		else if (key == 's') cv::waitKey(0);
	}
}