#pragma once
#include <vector>
#include <opencv2/opencv.hpp>
#include "dip_process.hpp"
#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "pill_detect.hpp"


void pub_vel(double v, double omega, bool isMove);

void pid_main(int argc, char **argv, cv::VideoCapture &capture);
