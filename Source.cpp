#include"mykinect_v2.h"
#include"NHKdebug.h"
#include"MySerial.h"
#include"xmlManage.h"
#include<iostream>

const int COMPORT = 5;
char ringtemp[1];

void robocon();
void saveMovie();
void useMovie();
void adjustValues();

//MySerial serial(COMPORT);

int main()
{
	ringtemp[0] = 'r';
	//M†‚ª‘—‚ç‚ê‚é‚Ü‚Å‘Ò‹@
	//serial.recieveData(ringtemp);

	//robocon();
	adjustValues();
	//saveMovie();
	//useMovie();
	return 0;
}

void robocon() {
	try {
		MyKinectV2 app;
		//ƒ|[ƒ‹‚Ìí—Ş‚ğŠi”[
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

void adjustValues() {
	try {
		KinectDebug dbg;
		dbg.poleLine.ringtype = ringtemp[0];
		dbg.initializeDepth();
		dbg.initializeColor();

		ValueManager<UINT16> depthManager("values.xml");
		depthManager.set_value("MINDEPTH", &dbg.MINDEPTH, 8000);
		depthManager.set_value("MAXDEPTH", &dbg.MAXDEPTH, 8000);
		depthManager.trackbar("depth");

		ValueManager<int> HSVManager("hsvValues.xml");
		HSVManager.set_value("min_h", &dbg.hsvKeeper.min_h, 180);
		HSVManager.set_value("min_s", &dbg.hsvKeeper.min_s, 255);
		HSVManager.set_value("min_v", &dbg.hsvKeeper.min_v, 255);
		HSVManager.set_value("max_h", &dbg.hsvKeeper.max_h, 180);
		HSVManager.set_value("max_s", &dbg.hsvKeeper.max_s, 255);
		HSVManager.set_value("max_v", &dbg.hsvKeeper.max_v, 255);
		HSVManager.trackbar("HSV");

		std::cout << "s: save values \nq: quit" << std::endl;
		while (1) {
			dbg.setDepth();
			dbg.setRGB();
			dbg.binarization(dbg.depthImage,dbg.MINDEPTH,dbg.MAXDEPTH);
			
			dbg.showDistance();
			dbg.showRGB();

			dbg.hsvKeeper.setHSVvalues();
			dbg.hsvKeeper.setHSVImage(dbg.RGBImage);
			dbg.hsvKeeper.extractColor();
			dbg.hsvKeeper.showHSVImage();

			auto key = cv::waitKey(1);
			if (key == 's') {
				depthManager.save_value();
				HSVManager.save_value();
				std::cout << "saved values!" << std::endl;
			}
			else if (key == 'q') {
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
		dbg.poleLine.ringtype = ringtemp[0];
		dbg.initializeDepth();
		dbg.initializeColor();
		while (1) {
			dbg.saveDepthMovie();
			dbg.saveRGBMovie();
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
		dbg.poleLine.ringtype = ringtemp[0];
		dbg.useDepthMovie();
	}
	catch (std::exception& ex) {
		std::cout << ex.what() << std::endl;
	}
}