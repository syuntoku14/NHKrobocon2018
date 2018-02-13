#include"mykinect_v2.h"
#include"process.h"
#include"MySerial.h"
#include"xmlManage.h"
#include<iostream>

//ìÆâÊÇÃï€ë∂êÊ
std::time_t now = std::time(nullptr);
std::string movieName_depth = "./shuttleMovies/depth" + std::to_string(now) + ".avi";
std::string movieName_RGB = "./shuttleMovies/RGB" + std::to_string(now) + ".avi";

const int COMPORT = 5;
char ringtemp[1];

int main()
{	
	//savemovie(moviename_rgb,moviename_depth);
	MyKinectV2 test;
	test.initializeColor();
	while (1) {
		if (!test.setRGBbyMovie("./successMovies/RGB1.avi")) break;
		cv::imshow("RGB", test.RGBImage);
		cv::waitKey(1);
	}
	return 0;
}
