#pragma once

#include <opencv2/opencv.hpp>
#include <vector>

bool GetLeftLinePoint(const cv::Mat &srcImg, double &value, double distance, double x_offset);
bool GetRightLinePoint(const cv::Mat &srcImg, double &value, double distance, double x_offset);

void DrawPoints(cv::Mat &srcImg, std::vector<cv::Point3i> points, int radius, const cv::Scalar &color, int thickness);

void GetLeftLinePoints(const cv::Mat &srcImg, std::vector<cv::Point3i> &points, double normlized_start_pos);
void GetRightLinePoints(const cv::Mat &srcImg, std::vector<cv::Point3i> &points, double normlized_start_pos);

void LinespaceY(std::vector<cv::Point3i> &points, double start, double end);