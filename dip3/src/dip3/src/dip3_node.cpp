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

#include <vector>

#include <cmath>

#define LINEAR_X 0

using namespace cv;

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

Mat EdgeDetector(Mat input)
{
    Mat tplt = (cv::Mat_<int>(3, 3) << 0, 1, 0, 1, -4, 1, 0, 1, 0);
    return ImgConvolve(input, tplt);
}

Mat Hough_Line(Mat input, int threshold)
{
    Mat result;
    
    // 进行边缘检测；
    Mat src_gray;
    Canny(input, src_gray, 150, 200, 3);
    cvtColor(src_gray, result, COLOR_GRAY2BGR);

    // 霍夫圆直线检测
    std::vector<Vec4f> plines;

    HoughLinesP(src_gray, plines, 1, CV_PI / 180, 10, 0, 10);
    for (size_t i = 0; i < plines.size(); i++)
    {
        Vec4f P = plines[i];
        // 绘制直线
        line(result, Point(P[0], P[1]), Point(P[2], P[3]), Scalar(0, 0, 255), 1);
    }
    return result;
}

Mat Hough_Circle(Mat input, double minDist = 25, double min_radius = 10, double max_radius = 200)
{
    Mat result;
    cvtColor(input, result, COLOR_GRAY2BGR);
    GaussianBlur(input, input, Size(3, 3), 2, 2);

    std::vector<Vec3f> circles;

    HoughCircles(input, circles, HOUGH_GRADIENT, 1, minDist, 70, 80, min_radius, max_radius);
    for (size_t t = 0; t < circles.size(); t++)
    {
        Point center(circles[t][0], circles[t][1]);
        circle(result, center, circles[t][2], Scalar(0, 0, 255), 2, 8, 0);
    }
    return result;
}

int main(int argc, char **argv)
{
    // VideoCapture capture;
    // capture.open(1); //打开 zed 相机
    ROS_WARN("*****START");
    ros::init(argc, argv, "trafficLaneTrack"); //初始化 ROS 节点
    ros::NodeHandle n;
    // ros::Rate loop_rate(10);//定义速度发布频率
    ros::Publisher pub = n.advertise<geometry_msgs::Twist>("/smoother_cmd_vel", 5); //定义速度发布器
    // if (!capture.isOpened())
    // {
    //     printf("摄像头没有正常打开，重新插拔工控机上当摄像头\n");
    //     return 0;
    // }
    waitKey(1000);
    Mat frame; //当前帧图片
    frame = imread("src/img/edge_detection.png");
    Mat line_detection = imread("src/img/line_detecton.png");
    Mat circle_img = imread("src/img/circle.png");
    // int nFrames = 0;                                      //图片帧数
    // int frameWidth = capture.get(CAP_PROP_FRAME_WIDTH);   //图片宽
    // int frameHeight = capture.get(CAP_PROP_FRAME_HEIGHT); //图片高
    while (ros::ok())
    {
        // capture.read(frame);
        if (frame.empty())
        {
            break;
        }
        Mat frIn = frame;
        // frIn = frame(cv::Rect(0, 0, frame.cols / 2, frame.rows)); //截取 zed 的左目图片

        // 灰度图转换
        Mat gray;
        cvtColor(frIn, gray, COLOR_BGR2GRAY);

        // 边缘检测函数
        Mat edge = EdgeDetector(gray);
        imshow("edge img", edge);

        // 线检测
        Mat line_detection_gray;
        cvtColor(line_detection, line_detection_gray, COLOR_BGR2GRAY);
        Mat line_detection_edge = EdgeDetector(line_detection_gray);
        imshow("line_detection_edge", line_detection_edge);
        Mat hough = Hough_Line(line_detection_gray, 100);
        imshow("Hough line", hough);

        // 圆检测
        Mat circle_gray;
        cvtColor(circle_img, circle_gray, COLOR_BGR2GRAY);
        Mat hough_circle = Hough_Circle(circle_gray);
        imshow("hough_circle", hough_circle);
        // imshow("1", output);
        // geometry_msgs::Twist cmd_red;
        // // 车的速度值设置
        // cmd_red.linear.x = LINEAR_X;
        // cmd_red.linear.y = 0;
        // cmd_red.linear.z = 0;
        // cmd_red.angular.x = 0;
        // cmd_red.angular.y = 0;
        // cmd_red.angular.z = 0.2;
        // pub.publish(cmd_red);
        ros::spinOnce();
        waitKey(5);
    }
    return 0;
}