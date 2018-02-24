#pragma once
#include"MyKinectV2.h"
#include"xmlManage.h"
#include<numeric>


void binarization(cv::Mat& image, const int minDepth, const int maxDepth) {
	double mid = 255 * minDepth / 8000.0, mxd = 255 * maxDepth / 8000.0;
	for (int i = 0; i < image.total(); ++i) {
		if (mid < image.data[i] && image.data[i] < mxd) {
			image.data[i] = 255;  //2’l‰»
		}
		else {
			image.data[i] = 0;
		}
	}
};

auto convBinarizaionByHsv(const cv::Mat &HSVImage, const cv::Mat &depthImage) {
	using namespace cv;
	Mat mask = Mat::zeros(HSVImage.rows, HSVImage.cols, CV_8UC1);
	int upper, mid, lower;
	for (int i = 0; i < HSVImage.total(); ++i) {
		if (HSVImage.data[i] > 200) {
			for (int delta = -1; delta < 2; delta++) {
				upper = i - HSVImage.cols + delta;
				mid = i + delta;
				lower = i + HSVImage.cols + delta;
				if (upper >= 0) mask.data[upper] = 1;
				if (mid >= 0 and mid < HSVImage.total()) mask.data[mid] = 1;
				if (lower < HSVImage.total()) mask.data[lower] = 1;
			}
		}
	}
	for (int i = 0; i < mask.total(); i++) {
		mask.data[i] = depthImage.data[i] * mask.data[i];
	}
	return mask;
}


void saveRGBandDepthMovies(std::string movieName_rgb, std::string movieName_depth) {
	try {
		MyKinectV2 kinect;

		kinect.initializeColor();
		kinect.initializeDepth();
		static cv::VideoWriter rgbWriter(movieName_rgb, cv::VideoWriter::fourcc('X', 'V', 'I', 'D'), 30, cv::Size(kinect.depthWidth, kinect.depthHeight), true);
		static cv::VideoWriter depthWriter(movieName_depth, cv::VideoWriter::fourcc('X', 'V', 'I', 'D'), 30, cv::Size(kinect.depthWidth, kinect.depthHeight), false);

		while (1) {
			kinect.setDepth();
			kinect.setMappedRGB();
			cv::Mat img;
			cv::cvtColor(kinect.RGBImage, img, CV_BGRA2BGR);
			depthWriter << kinect.depthImage;
			rgbWriter << img;
			cv::imshow("depthImage", kinect.depthImage);
			cv::imshow("RGBImage", kinect.RGBImage);

			auto key = cv::waitKey(1);
			if (key == 'q') {
				break;
			}
		}
	}
	catch (std::exception& ex) {
		std::cout << ex.what() << std::endl;
	}
};
