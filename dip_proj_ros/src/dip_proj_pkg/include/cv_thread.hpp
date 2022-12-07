#pragma once
#include <opencv2/opencv.hpp>
#include <thread>
#include <atomic>
#include <mutex>

enum class CV_STATE
{
    LineSearching,
    PillDetecting,
    Pause,
    Stop
};

extern std::mutex MyMutex;
extern std::atomic<CV_STATE> CvState;

extern double sender_endlineDistanceNormlized;
extern std::vector<cv::Vec2f> sender_endlines;
extern std::vector<cv::Point3d> sendleftLine;
extern std::vector<cv::Point3d> sendrightLine;
extern std::vector<cv::Point3d> sendmiddleLine;
extern double sender_compactness;
extern double sender_eccentricity;
// extern std::atomic_bool dip_main_running;

void CvThread();
