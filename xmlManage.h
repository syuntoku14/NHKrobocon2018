#pragma once

#include <iostream>
#include<opencv2\opencv.hpp>
#include<string>
#include<opencv2\highgui.hpp>
#include<fstream>

template<typename T>
class ValueManager {
public:
	cv::FileStorage fs;
	std::string filename;
	T* value;
	T originvalue;
	ValueManager(std::string filename, T* value) {
		this->filename = filename;
		this->value = value;
		this->originvalue = *value;
	};
	ValueManager() { fs.release() };

	void saveValue(std::string valuename) {
		using namespace cv;
		fs.open(filename, FileStorage::WRITE);
		fs << valuename << *value;
	};

	void getValue(std::string valuename) {
		using  namespace cv;
		fs.open(filename, FileStorage::READ);
		fs[valuename] >> *value;
	};

	int slider = 0, SLIDER_MAX = 100;

	static void onTrackbar(int, void *object) {
		ValueManager *self = (ValueManager*)object;
		*self->value = self->originvalue*((T)self->slider / self->SLIDER_MAX);
		std::cout << *self->value << std::endl;
	}

	void trackbar(std::string valuename) {
		using namespace cv;
		const std::string windowName = "track";
		int value = 10;
		namedWindow(windowName, CV_WINDOW_AUTOSIZE);
		createTrackbar(valuename, windowName, &slider, SLIDER_MAX, &ValueManager::onTrackbar, this);
	};
};