#pragma once

#include <opencv2/opencv.hpp>

cv::Mat ShowHSVSplitImg(const cv::Mat &srcImg, int minH, int maxH, int minS, int maxS, int minV, int maxV);
cv::Mat ShowHSVSplitImgTrackbar(const cv::Mat &srcImg);
cv::Mat HSVSplitImg(const cv::Mat &srcImg, int minH, int maxH, int minS, int maxS, int minV, int maxV);
void ColorEqualize(const cv::Mat &srcImg, cv::Mat &destImg);