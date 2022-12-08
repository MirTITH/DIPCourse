#pragma once
#include <vector>
#include <opencv2/opencv.hpp>
#include "dip_process.hpp"
#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "pill_detect.hpp"

enum class PID_CALC_STATE
{
    Running,
    PillDetecting,
    Stop
};

extern std::atomic<PID_CALC_STATE> PidCalcState;

void pid_main(ros::Publisher *vel_pub);
