#include <iostream>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/core/core.hpp>
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float32.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Image.h>
#include <cmath>

// 是否使用笔记本摄像头
#define USE_LAPTOP_CAMERA

using namespace cv;
using namespace std;

/**
 * @brief 计算高斯模板
 *
 * @param sigma
 * @return Mat 大小截取为 (3*sigma)*(3*sigma) CV_32FC1 类型
 */
Mat GaussianTemplate(double sigma)
{
    int templateSize = round(sigma * 3);
    templateSize = templateSize % 2 ? templateSize : templateSize + 1;
    Mat result(templateSize, templateSize, CV_32FC1);
    for (int i = -(templateSize - 1) / 2; i <= (templateSize - 1) / 2; i++)
    {
        for (int j = -(templateSize - 1) / 2; j <= (templateSize - 1) / 2; j++)
        {
            result.at<float>(i + (templateSize - 1) / 2, j + (templateSize - 1) / 2) = exp(-(double)(i * i + j * j) / (2 * sigma * sigma)) / (2 * M_PI * sigma * sigma);
        }
    }

    // 归一化
    result /= sum(result)[0];

    return result;
}

Mat ImgConvolve(Mat src, Mat kernel)
{
    Mat result, rotatedKernel;
    rotate(kernel, rotatedKernel, ROTATE_180);
    filter2D(src, result, -1, kernel, Point(-1, -1), (kernel.rows - 1) / 2, BORDER_CONSTANT);
    return result;
}

// 空域高斯滤波
Mat Gaussian(Mat input, double sigma)
{
    Mat tplt = GaussianTemplate(sigma);
    return ImgConvolve(input, tplt);
}

// 空域锐化
Mat Sharpen(Mat input)
{
    Mat tplt = (cv::Mat_<int>(3, 3) << 0, 1, 0, 1, -4, 1, 0, 1, 0);
    return ImgConvolve(input, tplt);
}

/**
 * @brief tplt 与 src 是否有交集
 *
 * @param src 二值矩阵(CV_8UC1)
 * @param tplt 二值矩阵(CV_8UC1)
 * @param src_pos
 * @param tplt_origin
 * @return true
 * @return false
 */
bool IsIntersection(Mat src, Mat tplt, Point src_pos, Point tplt_origin)
{
    for (int i = 0; i < tplt.rows; i++)
    {
        for (int j = 0; j < tplt.cols; j++)
        {
            if (tplt.at<uint8_t>(i, j) > 0)
            {
                int y = (src_pos.y - 1) + i - (tplt_origin.y - 1);
                int x = (src_pos.x - 1) + j - (tplt_origin.x - 1);
                if (x >= 0 && x < src.cols && y >= 0 && y < src.rows)
                {
                    if (src.at<uint8_t>(y, x) > 0)
                        return true;
                }
            }
        }
    }
    return false;
}

/**
 * @brief tplt 是否属于 src
 * 
 * @param src 
 * @param tplt 
 * @param src_pos 
 * @param tplt_origin 
 * @return true 
 * @return false 
 */
bool IsBelonging(Mat src, Mat tplt, Point src_pos, Point tplt_origin)
{
    for (int i = 0; i < tplt.rows; i++)
    {
        for (int j = 0; j < tplt.cols; j++)
        {
            if (tplt.at<uint8_t>(i, j) > 0)
            {
                int y = (src_pos.y - 1) + i - (tplt_origin.y - 1);
                int x = (src_pos.x - 1) + j - (tplt_origin.x - 1);
                if (x >= 0 && x < src.cols && y >= 0 && y < src.rows)
                {
                    if (src.at<uint8_t>(y, x) == 0)
                        return false;
                }
            }
        }
    }
    return true;
}

// 膨胀
Mat Dilate(Mat src)
{
    Mat dilateTplt = (cv::Mat_<uint8_t>(2, 2) << 0, 255, 255, 255);
    Mat rotatedTplt;
    rotate(dilateTplt, rotatedTplt, ROTATE_180);

    Mat result(src.rows, src.cols, CV_8UC1);

    for (int i = 0; i < src.rows; i++)
    {
        for (int j = 0; j < src.cols; j++)
        {
            result.at<uint8_t>(i, j) = IsIntersection(src, rotatedTplt, Point(j + 1, i + 1), Point(2, 2)) ? 255 : 0;
        }
    }

    return result;
}

// 腐蚀
Mat Erode(Mat src)
{
    Mat erodeTplt = (cv::Mat_<uint8_t>(3, 3) << 0, 255, 0, 255, 255, 255, 0, 255, 0);

    Mat result(src.rows, src.cols, CV_8UC1);

    for (int i = 0; i < src.rows; i++)
    {
        for (int j = 0; j < src.cols; j++)
        {
            result.at<uint8_t>(i, j) = IsBelonging(src, erodeTplt, Point(j + 1, i + 1), Point(2, 2)) ? 255 : 0;
        }
    }

    return result;
}

int main(int argc, char **argv)
{
    VideoCapture capture;

#ifdef USE_LAPTOP_CAMERA
    capture.open(0);
#else
    capture.open(1);
#endif

    ROS_WARN("****START");

    ros::init(argc, argv, "trafficLaneTrack");

    ros::NodeHandle n;
    // ros::Rate loop_rate(10);

    ros::Publisher pub = n.advertise<geometry_msgs::Twist>("/smoother_cmd_vel", 5);

    if (!capture.isOpened())
    {
        std::cout << "Failed to open camera." << std::endl;
        return 0;
    }

    waitKey(1000);

    Mat frame;
    int nFrames = 0;

    int frameWidth = capture.get(CAP_PROP_FRAME_WIDTH);
    int frameHeight = capture.get(CAP_PROP_FRAME_HEIGHT);

    while (ros::ok())
    {
        capture.read(frame);
        if (frame.empty())
        {
            break;
        }

#ifdef USE_LAPTOP_CAMERA
        Mat frIn(270, 480, CV_8UC3);
        resize(frame, frIn, frIn.size());
#else
        Mat frIn = frame(cv::Rect(0, 0, frame.cols / 2, frame.rows));
#endif
        // Erode();

        imshow("Source Frame", frIn);

        imshow("Gaussian", Gaussian(frIn, 5));
        imshow("Sharpen", Sharpen(frIn));

        Mat binaryFrame, greyFrame;

        cvtColor(frIn, greyFrame, COLOR_RGB2GRAY);

        threshold(greyFrame, binaryFrame, 127, 255, THRESH_BINARY| THRESH_OTSU);
        // adaptiveThreshold(greyFrame, binaryFrame, 255, ADAPTIVE_THRESH_GAUSSIAN_C,THRESH_BINARY, 55, 0);

        imshow("Binary Frame",binaryFrame);
        imshow("Dilate", Dilate(binaryFrame));
        imshow("Erode", Erode(binaryFrame));

        ros::spinOnce();

        waitKey(5);
    }
    return 0;
}
