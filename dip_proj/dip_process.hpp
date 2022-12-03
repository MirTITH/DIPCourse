#pragma once
#include <opencv2/opencv.hpp>
#include <vector>

void NormalMode(cv::Mat leftMask, cv::Mat rightMask, cv::Mat leftBGR, cv::Mat rightBGR);

void SplitFrame(const cv::Mat &srcFrame, cv::Mat &leftFrame, cv::Mat &rightFrame);
cv::Mat MergeFrame(const cv::Mat &leftFrame, const cv::Mat &rightFrame);
void dip_main();