#pragma once

#include <iostream>
#include<opencv2\opencv.hpp>
#include<string>
#include<opencv2\highgui.hpp>
#include<fstream>
#include<map>

template<typename T>
class ValueManager {
public:
#pragma region donttouch
	class V {
	public:
		int slider;
		T* value;
		T max_value;
	};
	cv::FileStorage fs;
	std::string filename;
	std::map<std::string, V> values;
	ValueManager(std::string filename) {
		this->filename = filename;
	};

	~ValueManager() {
		fs.release();
	};
	
	static void onTrackbar(int, void *object) {
		using namespace std;
		V *val = (V*)object;
		*val->value = (T)val->slider;
	};
#pragma endregion

#pragma region functions
	void get_value(std::string value_name, T *value) {
		using  namespace cv;
		fs.open(filename, FileStorage::READ);
		fs[value_name] >> *value;
		fs.release();
	};

	void set_value(std::string value_name, T* value, T max_value) {
		V val;
		val.slider = 0;
		get_value(value_name,value);
		std::cout << *value << std::endl;
		val.value = value;
		val.max_value = max_value;
		values[value_name] = val;
	};

	void save_value() {
		using namespace cv;
		fs.open(filename, FileStorage::WRITE);
		while (!values.empty()) {
			auto itr = values.begin();
			fs << itr->first << *itr->second.value;
			values.erase(values.begin());
		}
	};

	void trackbar(std::string windowName) {
		using namespace cv;
		cv::namedWindow(windowName, CV_WINDOW_AUTOSIZE);
		for (auto i = values.begin(); i != values.end(); ++i) {
			createTrackbar(i->first, windowName, &i->second.slider, (int)i->second.max_value, &ValueManager::onTrackbar, &i->second);
		}
	};
#pragma endregion

};