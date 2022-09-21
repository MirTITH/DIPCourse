#include <stdlib.h>
#include <iostream>
#include <string>
#include <algorithm>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include "ros/ros.h"
#define READIMAGE_ONLY
#ifndef READIMAGE_ONLY
#include <geometry_msgs/Twist.h>
#endif

using namespace cv;
using namespace std;

void calcCumulativeDensity(vector<int> &cumulativeDensity, Mat &grey)
{
    //计算各灰度级像素个数
    vector<int> nums(256);
    for (int i = 0; i < grey.rows; i++)
    {
        uchar *p = grey.ptr<uchar>(i);
        for (int j = 0; j < grey.cols; j++)
        {
            nums[p[j]]++;
        }
    }
    float temp = 0;

    for (int i = 0; i < 256; i++)
    {
        temp += 255.0f * nums[i] / grey.size().area();
        cumulativeDensity[i] = cvRound(temp);
    }
}

void drawDensity(vector<int> nums)
{
    Mat hist = Mat::zeros(600, 800, CV_8UC3);
    auto Max = max_element(nums.begin(), nums.end()); // max迭代器类型,最大数目
    putText(hist, "Density", Point(150, 100), FONT_HERSHEY_DUPLEX, 1, Scalar(255, 255, 255));
    //*********绘制坐标系************//
    Point o = Point(100, 550);
    Point x = Point(700, 550);
    Point y = Point(100, 150);
    // x轴
    line(hist, o, x, Scalar(255, 255, 255), 2, 8, 0);
    // y轴
    line(hist, o, y, Scalar(255, 255, 255), 2, 8, 0);

    //********绘制灰度曲线***********//
    Point pts[256];
    //生成坐标点
    for (int i = 0; i < 256; i++)
    {
        pts[i].x = i * 2 + 100;
        pts[i].y = 550 - int(nums[i] * (300.0 / (*Max))); //归一化到[0, 300]
        //显示横坐标
        if ((i + 1) % 16 == 0)
        {
            string num = format("%d", i + 1);
            putText(hist, num, Point(pts[i].x, 570), FONT_HERSHEY_SIMPLEX, 0.5, Scalar(255, 255, 255));
        }
    }
    //绘制线
    for (int i = 1; i < 256; i++)
    {
        line(hist, pts[i - 1], pts[i], Scalar(0, 255, 0), 2);
    }
    //显示图像
    imshow("累计密度", hist);
}

void openCVHist(const Mat src)
{
    //需要计算图像的哪个通道（bgr空间需要确定计算 b或g或r空间）
    const int channels[1] = {0};
    //直方图的每一个维度的 柱条的数目（就是将灰度级分组）
    int histSize[] = {256}; //如果这里写成int histSize = 256; 那么下面调用计算直方图的函数的时候，该变量需要写 &histSize
                            //定义一个变量用来存储 单个维度 的数值的取值范围
    float midRanges[] = {0, 256};
    //确定每个维度的取值范围，就是横坐标的总数
    const float *ranges[] = {midRanges};
    //输出的结果存储的 空间 ，用MatND类型来存储结果
    MatND dstHist;

    calcHist(&src, 1, channels, Mat(), dstHist, 1, histSize, ranges, true, false);

    // calcHist  函数调用结束后，dstHist变量中将储存了直方图的信息, 用dstHist的模版函数 at<Type>(i)得到第i个柱条的值  at<Type>(i, j)得到第i个并且第j个柱条的值
    //首先先创建一个黑底的图像，为了可以显示彩色，所以该绘制图像是一个8位的3通道图像
    Mat drawImage = Mat::zeros(Size(256, 256), CV_8UC3);

    //一个图像的某个灰度级的像素个数（最多为图像像素总数），可能会超过显示直方图的所定义的图像的尺寸，因此绘制直方图的时候，让直方图最高的地方只有图像高度的90%来显示
    //先用minMaxLoc函数来得到计算直方图后的像素的最大个数
    double g_dHistMaxValue;
    minMaxLoc(dstHist, 0, &g_dHistMaxValue, 0, 0);

    //遍历直方图得到的数据
    for (int i = 0; i < 256; i++)
    {
        int value = cvRound(256 * 0.9 * (dstHist.at<float>(i) / g_dHistMaxValue));
        line(drawImage, Point(i, drawImage.rows - 1), Point(i, drawImage.rows - 1 - value), Scalar(255, 0, 0));
    }
    imshow("OpenCVHist", drawImage);
    waitKey(0);
}

//直方图绘制函数，参数vector<int> nums 是灰度图片256级灰度的像素个数
void drawHist(vector<int> nums, const string title)
{
    Mat hist = Mat::zeros(600, 800, CV_8UC3);
    auto Max = max_element(nums.begin(), nums.end()); // max迭代器类型,最大数目
    putText(hist, "Histogram", Point(150, 100), FONT_HERSHEY_DUPLEX, 1, Scalar(255, 255, 255));
    //*********绘制坐标系************//
    Point o = Point(100, 550);
    Point x = Point(700, 550);
    Point y = Point(100, 150);
    // x轴
    line(hist, o, x, Scalar(255, 255, 255), 2, 8, 0);
    // y轴
    line(hist, o, y, Scalar(255, 255, 255), 2, 8, 0);

    //********绘制灰度曲线***********//
    Point pts[256];
    //生成坐标点
    for (int i = 0; i < 256; i++)
    {
        pts[i].x = i * 2 + 100;
        pts[i].y = 550 - int(nums[i] * (300.0 / (*Max))); //归一化到[0, 300]
        //显示横坐标
        if ((i + 1) % 16 == 0)
        {
            string num = format("%d", i + 1);
            putText(hist, num, Point(pts[i].x, 570), FONT_HERSHEY_SIMPLEX, 0.5, Scalar(255, 255, 255));
        }
    }
    //绘制线
    for (int i = 1; i < 256; i++)
    {
        line(hist, pts[i - 1], pts[i], Scalar(0, 255, 0), 2);
    }
    //显示图像
    imshow(title, hist);
}
//计算直方图，统计各灰度级像素个数
void calHist(const Mat grey, const string title)
{
    //计算各灰度级像素个数
    vector<int> nums(256);
    for (int i = 0; i < grey.rows; i++)
    {
        const uchar *p = grey.ptr<uchar>(i);
        for (int j = 0; j < grey.cols; j++)
        {
            nums[p[j]]++;
        }
    }
    drawHist(nums, title);
}

int main(int argc, char **argv)
{
    ROS_WARN("*****START*****");
    ros::init(argc, argv, "trafficLaneTrack"); //初始化ROS节点
    ros::NodeHandle n;

    // Before the use of camera, you can test ur program with images first: imread()
    VideoCapture capture;
    capture.open(0); //打开zed相机，如果要打开笔记本上的摄像头，需要改为0
    waitKey(100);
    if (!capture.isOpened())
    {
        printf("摄像头没有正常打开，重新插拔工控机上当摄像头\n");
        return 0;
    }

#ifndef READIMAGE_ONLY
    ros::Publisher pub = n.advertise<geometry_msgs::Twist>("/smoother_cmd_vel", 5); //定义dashgo机器人的速度发布器
#endif
    Mat src_frame;
    while (ros::ok())
    {
        capture.read(src_frame);
        if (src_frame.empty())
        {
            break;
        }
        imshow("src", src_frame);

        Mat grey;
        //先转为灰度图
        cvtColor(src_frame, grey, COLOR_BGR2GRAY);
        imshow("灰度图", grey);

        calHist(grey, "直方图");

        vector<int> cumulativeDensity(256);

        Mat equ_grey;

        cvtColor(src_frame, equ_grey, COLOR_BGR2GRAY);

        calcCumulativeDensity(cumulativeDensity, equ_grey);

        drawDensity(cumulativeDensity);

        // printf("%d\n", cumulativeDensity[100]);

        for (int y = 1; y < equ_grey.rows; y++)
        {
            for (int x = 1; x < equ_grey.cols; x++)
            {
                equ_grey.at<uint8_t>(y, x) = cumulativeDensity[equ_grey.at<uint8_t>(y, x)];
            }
        }

        imshow("均衡化后的灰度图", equ_grey);

        calHist(equ_grey, "均衡化直方图");

#ifndef READIMAGE_ONLY
        //以下代码可设置机器人的速度值，从而控制机器人运动
        geometry_msgs::Twist cmd_red;
        cmd_red.linear.x = 0;
        cmd_red.linear.y = 0;
        cmd_red.linear.z = 0;
        cmd_red.angular.x = 0;
        cmd_red.angular.y = 0;
        cmd_red.angular.z = 0.2;
        pub.publish(cmd_red);
#endif
        ros::spinOnce();
        waitKey(5);
    }
    return 0;
}
