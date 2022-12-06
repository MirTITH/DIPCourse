#pragma once
#include <vector>
#include <opencv2/opencv.hpp>
#include "dip_process.hpp"

double diffCalc(std::vector<cv::Point3d> leftLine,std::vector<cv::Point3d> rightLine,std::vector<cv::Point3d> middleLine,int mode);
