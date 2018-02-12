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
	saveMovie(movieName_RGB,movieName_depth);
	return 0;
}
