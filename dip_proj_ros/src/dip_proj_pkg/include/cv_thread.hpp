#pragma once
#include <opencv2/opencv.hpp>
#include <thread>
#include <atomic>
#include <mutex>

enum class CV_STATE
{
    LineSearching,
    Detecting,
    Pause,
    Stop
};

extern std::mutex MyMutex;
extern std::atomic<CV_STATE> CvState;

extern std::vector<cv::Point3d> kLeftLine;
extern std::vector<cv::Point3d> kRightLine;
extern std::vector<cv::Point3d> kMiddleLine;

void CvThread();
