#include"mykinect_v2.h"
#include"NHKdebug.h"
#include"MySerial.h"
#include<iostream>

const int COMPORT = 5;
char ringtemp[1];

void robocon();
void saveMovie();
void useMovie();

//MySerial serial(COMPORT);

int main()
{
	ringtemp[0] = 'r';
	//M†‚ª‘—‚ç‚ê‚é‚Ü‚Å‘Ò‹@
	//serial.recieveData(ringtemp);

	//robocon();
	//saveMovie();
	useMovie();
	return 0;
}

void robocon() {
	try {
		MyKinectV2 app;
		//ƒ|[ƒ‹‚Ìí—Ş‚ğŠi”[
		app.ringtype = ringtemp[0];
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

void saveMovie() {
	try {
		KinectDebug dbg;
		//ƒ|[ƒ‹‚Ìí—Ş‚ğŠi”[
		dbg.ringtype = ringtemp[0];
		dbg.initializeDepth();
		while (1) {
			dbg.saveDepthMovie();
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

void useMovie() {
	try {
		KinectDebug dbg;
		dbg.initializeDepth();
		dbg.ringtype = ringtemp[0];
		dbg.useDepthMovie();
	}
	catch (std::exception& ex) {
		std::cout << ex.what() << std::endl;
	}
}