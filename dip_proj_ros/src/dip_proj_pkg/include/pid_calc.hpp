#pragma once
#include <vector>
#include <opencv2/opencv.hpp>
#include "dip_process.hpp"
#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "pill_detect.hpp"

double diffCalc(std::vector<cv::Point3d> leftLine,std::vector<cv::Point3d> rightLine,std::vector<cv::Point3d> middleLine,int mode);

void pub_vel(double v, double omega, bool isMove);

void pid_main(int argc, char **argv);
