#pragma once
#include <opencv2/opencv.hpp>
#include <vector>
#define CameraNumber 0
#define EdgePointNum 100

void NormalMode(cv::Mat leftMask, cv::Mat rightMask, cv::Mat leftBGR, cv::Mat rightBGR);
void SplitFrameLeft(const cv::Mat &srcFrame, cv::Mat &leftFrame);
void SplitFrame(const cv::Mat &srcFrame, cv::Mat &leftFrame, cv::Mat &rightFrame);
cv::Mat MergeFrame(const cv::Mat &leftFrame, const cv::Mat &rightFrame);
void dip_main(cv::VideoCapture *capture);