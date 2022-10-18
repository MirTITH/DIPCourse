#include <stdlib.h>
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

#define LINEAR_X 0

using namespace cv;

void EdgeDetector(Mat input, Mat output)
{
}

Mat Hough_Line(Mat output)
{
}

Mat Hough_Circle(Mat output)
{
}

int main(int argc, char **argv)
{
    VideoCapture capture;
    capture.open(1); //打开 zed 相机
    ROS_WARN("*****START");
    ros::init(argc, argv, "trafficLaneTrack"); //初始化 ROS 节点
    ros::NodeHandle n;
    // ros::Rate loop_rate(10);//定义速度发布频率
    ros::Publisher pub = n.advertise<geometry_msgs::Twist>("/smoother_cmd_vel", 5); //定义速度发布器
    if (!capture.isOpened())
    {
        printf("摄像头没有正常打开，重新插拔工控机上当摄像头\n");
        return 0;
    }
    waitKey(1000);
    Mat frame;                                               //当前帧图片
    int nFrames = 0;                                         //图片帧数
    int frameWidth = capture.get(CV_CAP_PROP_FRAME_WIDTH);   //图片宽
    int frameHeight = capture.get(CV_CAP_PROP_FRAME_HEIGHT); //图片高
    while (ros::ok())
    {
        capture.read(frame);
        if (frame.empty())
        {
            break;
        }
        Mat frIn = frame(cv::Rect(0, 0, frame.cols / 2, frame.rows)); //截取 zed 的左目图片
        // 灰度图转换
        CvtColor()
            // 边缘检测函数
            EdgeDetector();
        // 线检测
        Hough_Line();
        // 圆检测
        Hough_Circle();
        imshow("1", output);
        geometry_msgs::Twist cmd_red;
        // 车的速度值设置
        cmd_red.linear.x = LINEAR_X;
        cmd_red.linear.y = 0;
        cmd_red.linear.z = 0;
        cmd_red.angular.x = 0;
        cmd_red.angular.y = 0;
        cmd_red.angular.z = 0.2;
        pub.publish(cmd_red);
        ros::spinOnce();
        waitKey(5);
    }
    return 0;
}