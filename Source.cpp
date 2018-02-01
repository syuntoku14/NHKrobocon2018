#include"mykinect_v2.h"
#include"MySerial.h"
#include<iostream>

const int COMPORT = 5;
char ringtemp[1];

void robocon();
void saveMovie();
void useMovie();

MySerial serial(COMPORT);

void main()
{
	ringtemp[0] = 'r';
	//信号が送られるまで待機
	//serial.recieveData(ringtemp);

	//robocon();
	//saveMovie();
	useMovie();

}

void robocon() {
	try {
		MyKinectV2 app;
		//ポールの種類を格納
		app.ringtype = ringtemp[0];
		app.initializeDepth();
		while (1) {
			app.updateDepth();
			app.drawDepth();
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
		MyKinectV2 app;
		//ポールの種類を格納
		app.ringtype = ringtemp[0];
		app.initializeDepth();
		while (1) {
			app.updateDepth();
			app.saveDepthMovie();
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
		MyKinectV2 app;
		app.initializeDepth();
		//ポールの種類を格納
		app.ringtype = ringtemp[0];
		app.useDepthMovie();
	}
	catch (std::exception& ex) {
		std::cout << ex.what() << std::endl;
	}
}