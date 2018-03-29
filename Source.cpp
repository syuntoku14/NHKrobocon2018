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
�|�[���y�уV���g���R�b�N�̍��W�f�[�^���}�C�R���ƒʐM����
1�D�}�C�R���̓V���A���ʐM�Ń|�[���̐F(�ԂȂ�'r', ���F�Ȃ�'g')�𑗐M���遨�v���O�������X�^�[�g����
	�X�^�[�g�Ɠ����ɁA�}�C�R���Ƀ|�[���̐F��Ԃ�('r'��'g')�B�@�}�C�R���͂�������ăV���g���R�b�N���m���X�^�[�g�������Ƃ�F������B
2�DPC�̓V���A���ʐM�Ń|�[����Kinect����̂���p�𑗂葱����B�}�C�R�����͂��̃f�[�^�����ƂɊp�x���C�����A
�C�����I�������V���A����'q'�𑗂�B�󂯎�莟��PC�̓V���g���R�b�N��������Ɉڂ�
3. PC�̓V���g���R�b�N��������(���s�Ȃ�'f',�����Ȃ�'s')�𑗂葱����B���������ffffffffsssssss...�̂悤�ɁA�ŏ��͕K��f�𑗂葱���A���������^�C�~���O��s�𑗂�t����B
�}�C�R����PC�̃v���O�������I�����������^�C�~���O��'q'�𑗂�B�i�v���O���������[�v�𔲂��ďI������B
*/

//����̕ۑ���
std::time_t now = std::time(nullptr);
std::string movieName_depth = "./shuttleMovies/depth" + std::to_string(now) + ".avi";
std::string movieName_RGB = "./shuttleMovies/RGB" + std::to_string(now) + ".avi";

void test();
void LSDtestByMovie(char &ringtype);
void LSDtestByKinect(char &ringtype);
void adjustValues(std::string movieName_rgb, std::string movieName_depth);

//��{�I�Ɏ��Ă��Ȃ��f�[�^�ɂ�-1������

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
	//serial�ʐM��ringtype���󂯎������s���J�n
	serial.recieveData(ringtype);
	cout << ringtype[0] << endl;
	serial.sendData(ringtype[0]);
	serial.~MySerial();
	Sleep(1000); //�󂯎������1�b�҂�

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

		//�摜�����p�[�g
		convedImage = convBinarizaionByHsv(kinect.hsvKeeper.hsvImage, kinect.depthImage); //�Ή�����F�̕��������[�x��\����������
		//�������o
		setPoleDatabyLSD(convedImage, poledata, 0, 0.90);
		setPoleDepth(poledata, kinect.depthImage, kinect.hsvKeeper.hsvImage); //�|�[���̐[�x���i�[
		showPoleLine(kinect.depthImage, poledata);

		ringHSV.setHSVvalues();
		ringHSV.setHSVImage(kinect.RGBImage);
		ringHSV.extractColor();
		convedRing = convBinarizaionByHsv(kinect.depthImage, ringHSV.hsvImage); //�F�t�߂������o��������

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
		convedImage = convBinarizaionByHsv(kinect.hsvKeeper.hsvImage, kinect.depthImage); //�ԐF�t�߂������o��������
		ringHSV.setHSVvalues();
		ringHSV.setHSVImage(kinect.RGBImage);
		ringHSV.extractColor();
		convedRing = convBinarizaionByHsv(kinect.depthImage, ringHSV.hsvImage); //�F�t�߂������o��������
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
