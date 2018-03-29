#include"MyKinectV2.h"
#include"image_processing.h"
#include"xmlManage.h"
#include<iostream>
#include<numeric>
#include"PoleAndShuttlePrediction.h"
#include"MySerial.h"
#include<thread>
#include<string>

#define COMPORT 6 
/**
ポール及びシャトルコックの座標データをマイコンと通信する
1．マイコンはシリアル通信でポールの色(赤なら'r', 黄色なら'g')を送信する→プログラムがスタートする
	スタートと同時に、マイコンにポールの色を返す('r'か'g')。　マイコンはこれを見てシャトルコック検知がスタートしたことを認識する。
2．PCはシリアル通信でポールのKinectからのずれ角を送り続ける。マイコン側はそのデータをもとに角度を修正し、
修正が終わったらシリアルで'q'を送る。受け取り次第PCはシャトルコック成功判定に移る
3. PCはシャトルコック成功判定(失敗なら'f',成功なら's')を送り続ける。成功判定はffffffffsssssss...のように、最初は必ずfを送り続け、成功したタイミングでsを送り付ける。
マイコンはPCのプログラムを終了させたいタイミングで'q'を送る。（プログラムがループを抜けて終了する。
*/

//動画の保存先
std::time_t now = std::time(nullptr);
std::string movieName_depth = "./shuttleMovies/depth" + std::to_string(now) + ".avi";
std::string movieName_RGB = "./shuttleMovies/RGB" + std::to_string(now) + ".avi";

void test();
void LSDtestByMovie(char &ringtype);
void LSDtestByKinect(char &ringtype);
void adjustValues(std::string movieName_rgb, std::string movieName_depth);

//基本的に取れていないデータには-1が入る

MySerial serial(COMPORT);

//
void  rcv_char(char &c) {
	using namespace std;
	MySerial serial(3);
	while (1) {
		char buffer[8];
		serial.recieveData(buffer);
		c = buffer[0];
		cout << buffer << endl;
	}
}

void rcv_string(std::string &str) {
	MySerial serial(3);
	while (1) {
		char buffer[8];
		serial.recieveData(buffer);
		str = std::string(buffer);
	}
}

int main() {
	using namespace std;
	char ringtype[1];
	//serial通信でringtypeを受け取ったら行動開始
	serial.recieveData(ringtype);
	cout << ringtype[0] << endl;
	serial.sendData(ringtype[0]);
	serial.~MySerial();
	Sleep(1000); //受け取ったら1秒待つ

	//saveRGBandMappedDepthMovies(movieName_RGB,movieName_depth);
	//LSDtestByKinect(ringtype[0]);
	LSDtestByMovie(ringtype[0]);
	//adjustValues("./shuttleMovies/rgb_success.avi", "./shuttleMovies/depth_success.avi");
	return 0;
}

void LSDtestByMovie(char &ringtype) {
	using namespace std;
	PoleData poledata;
	poledata.ringtype = ringtype;

	MyKinectV2 kinect;
	kinect.initializeColor();
	kinect.initializeDepth();
	cv::Mat convedImage, convedRing;
	kinect.hsvKeeper.initHSVvalues("hsvValues_red.xml");
	HSVkeeper ringHSV;
	ringHSV.initHSVvalues("hsvValues_blue.xml");

	char msg = 'y';
	//std::thread thd(rcv_char, std::ref(msg));
	//thd.detach();

	while (1) {
		if (!kinect.setRGBbyMovie("./shuttleMovies/rgb_success.avi")) break;
		kinect.setDepthbyMovie("./shuttleMovies/depth_success.avi");
		cv::resize(kinect.RGBImage, kinect.RGBImage, cv::Size(512, 424));
		cv::resize(kinect.depthImage, kinect.depthImage, cv::Size(512, 424));
		cv::imshow("RGB", kinect.RGBImage);
		kinect.hsvKeeper.setHSVvalues();
		kinect.hsvKeeper.setHSVImage(kinect.RGBImage);
		kinect.hsvKeeper.extractColor();
		cv::imshow("HSVImage", kinect.hsvKeeper.hsvImage);
		cv::imshow("depthImage", kinect.depthImage);

		//画像処理パート
		convedImage = convBinarizaionByHsv(kinect.hsvKeeper.hsvImage, kinect.depthImage); //対応する色の部分だけ深度を表示したもの
		//直線検出
		setPoleDatabyLSD(convedImage, poledata, 0, 0.90);
		setPoleDepth(poledata, kinect.depthImage, kinect.hsvKeeper.hsvImage); //ポールの深度を格納
		showPoleLine(kinect.depthImage, poledata);

		ringHSV.setHSVvalues();
		ringHSV.setHSVImage(kinect.RGBImage);
		ringHSV.extractColor();
		convedRing = convBinarizaionByHsv(kinect.depthImage, ringHSV.hsvImage); //青色付近だけ抽出したもの

		poledata.setpole_angle();
		if (poledata.found_angle_flag) {
			if (msg == 'y') {
				cout << "pole angle" << (int)poledata.pole_angle << endl;
				serial.sendData(poledata.pole_angle);
			}
		}

		//if (msg == 'q') {
			find_shuttleLoc(poledata, convedRing);
			cout << poledata.success_flag << endl;
		//}

		auto key = cv::waitKey(1);
		if (key == 'q') break;
		else if (key == 's') cv::waitKey(0);
	}
}

void LSDtestByKinect(char &ringtype) {

	using namespace std;
	PoleData poledata;
	poledata.ringtype = ringtype;

	MyKinectV2 kinect;
	kinect.initializeColor();
	kinect.initializeDepth();
	kinect.initializeMulti();
	cv::Mat convedImage, convedRing;
	kinect.hsvKeeper.initHSVvalues("hsvValues_red.xml");
	HSVkeeper ringHSV;
	ringHSV.initHSVvalues("hsvValues_blue.xml");

	while (1) {
		kinect.setMappedDepthandRGB();
		kinect.hsvKeeper.setHSVvalues();
		kinect.hsvKeeper.setHSVImage(kinect.RGBImage);
		kinect.hsvKeeper.extractColor();
		convedImage = convBinarizaionByHsv(kinect.hsvKeeper.hsvImage, kinect.depthImage); //赤色付近だけ抽出したもの
		ringHSV.setHSVvalues();
		ringHSV.setHSVImage(kinect.RGBImage);
		ringHSV.extractColor();
		convedRing = convBinarizaionByHsv(kinect.depthImage, ringHSV.hsvImage); //青色付近だけ抽出したもの
		cv::imshow("rgb", kinect.RGBImage);
		//cv::imshow("convedRing", convedRing);
		setPoleDatabyLSD(convedImage, poledata, 0, 0.90);
		setPoleDepth(poledata, kinect.depthImage, kinect.hsvKeeper.hsvImage);
		showPoleLine(kinect.depthImage, poledata);

		find_shuttleLoc(poledata, convedRing);
		poledata.setpole_angle();
		if (poledata.found_angle_flag) {
			cout << "pole angle" << (int)poledata.pole_angle << endl;
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
	kinect.initializeMulti();
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
			kinect.setMappedDepthandRGB();
			break;
		}

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
