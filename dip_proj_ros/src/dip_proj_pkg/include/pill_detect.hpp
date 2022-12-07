#pragma once
#include <opencv2/opencv.hpp>
#include <vector>

// int pill_detect_main(cv::VideoCapture &cap);

void pill_detect_loop(cv::VideoCapture &cap, double &compactness, double &eccentricity);
