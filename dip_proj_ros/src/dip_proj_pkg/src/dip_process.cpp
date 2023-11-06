#include "dip_process.hpp"
#include "line_detect.hpp"
#include "color_split.hpp"
#include <thread>
#include <atomic>
#include <mutex>
#include "cv_thread.hpp"

using namespace std;
using namespace cv;

void SplitFrame(const Mat &srcFrame, Mat &leftFrame, Mat &rightFrame)
{
    auto width = srcFrame.cols;
    auto height = srcFrame.rows;

    Rect leftRect(0, 0, width / 2, height);
    Rect rightRect(width / 2, 0, width / 2, height);

    leftFrame = srcFrame(leftRect);
    rightFrame = srcFrame(rightRect);
}

void SplitFrameLeft(const Mat &srcFrame, Mat &leftFrame)
{
    auto width = srcFrame.cols;
    auto height = srcFrame.rows;
    Rect leftRect(0, 0, width / 2, height);
    leftFrame = srcFrame(leftRect);
}

Mat MergeFrame(const Mat &leftFrame, const Mat &rightFrame)
{
    Mat resultImg = Mat(leftFrame.rows, leftFrame.cols + rightFrame.cols, CV_8UC3, Scalar::all(0));
    Mat ROI_1 = resultImg(Rect(0, 0, leftFrame.cols, leftFrame.rows));
    Mat ROI_2 = resultImg(Rect(leftFrame.cols, 0, rightFrame.cols, rightFrame.rows));
    leftFrame.copyTo(ROI_1);
    rightFrame.copyTo(ROI_2);
    return resultImg;
}

void SearchLine(const Mat leftMask, const Mat rightMask, Mat leftBGR, Mat rightBGR, std::vector<Point3i> &leftPoints, std::vector<Point3i> &rightPoints, std::vector<Point3i> &middlePoints)
{
    auto left_thread = thread([&]()
                              {
        LinespaceY(leftPoints, leftMask.rows - 1, 0);
        GetLeftLinePoints(leftMask, leftPoints, 0.5); });

    auto right_thread = thread([&]()
                               {
        LinespaceY(rightPoints, rightMask.rows - 1, 0);
        GetRightLinePoints(rightMask, rightPoints, 0.5); });

    left_thread.join();
    right_thread.join();

    DrawPoints(leftBGR, leftPoints, 2, Scalar(255, 100, 100), 2);
    DrawPoints(leftBGR, rightPoints, 2, Scalar(100, 100, 255), 2);

    for (int i = 0; i < EdgePointNum; i++)
    {
        middlePoints[i] = (leftPoints[i] + rightPoints[i]) / 2;
    }

    DrawPoints(leftBGR, middlePoints, 2, Scalar(100, 255, 100), 2);

    imshow("EdgePoints", leftBGR);
}

void DrawLines(Mat &img,            // 要标记直线的图像
               vector<Vec2f> lines, // 检测的直线数据
               Scalar color,        // 绘制直线的颜色
               int thickness        // 绘制直线的线宽
)
{
    Point pt1, pt2;
    for (size_t i = 0; i < lines.size(); i++)
    {
        float rho = lines[i][0];                 // 直线距离坐标原点的距离
        float theta = lines[i][1];               // 直线过坐标原点垂线与x轴夹角
        double a = cos(theta);                   // 夹角的余弦值
        double b = sin(theta);                   // 夹角的正弦值
        double x0 = a * rho, y0 = b * rho;       // 直线与过坐标原点的垂线的交点
        double length = max(img.rows, img.cols); // 图像高宽的最大值
                                                 // 计算直线上的一点
        pt1.x = cvRound(x0 + length * (-b));
        pt1.y = cvRound(y0 + length * (a));
        // 计算直线上另一点
        pt2.x = cvRound(x0 - length * (-b));
        pt2.y = cvRound(y0 - length * (a));
        // 两点绘制一条直线
        line(img, pt1, pt2, color, thickness);
    }
}

double CalcLinesAvgDistance(const Mat &img, vector<Vec2f> lines)
{
    double distance = 0;
    for (auto line : lines)
    {
        double rho = line[0];   // 直线距离坐标原点的距离
        double theta = line[1]; // 直线过坐标原点垂线与x轴夹角
        distance += (rho / sin(theta) - img.cols / tan(theta) / 2) / lines.size();
    }
    return distance;
}

void EndLineDetect(const Mat &BinaryImg, double &distance, vector<Vec2f> &lines)
{
    Mat temp;
    BinaryImg.copyTo(temp);
    cvtColor(temp, temp, COLOR_GRAY2BGR);
    HoughLines(BinaryImg, lines, 1, CV_PI / 360, 200, 0, 0, 80.0 / 180.0 * CV_PI, 100.0 / 180.0 * CV_PI);
    distance = CalcLinesAvgDistance(BinaryImg, lines);
    string lineNumText = to_string(distance);
    string lineNumSizeText = to_string(lines.size());
    putText(temp, lineNumText, temp.size() / 2, cv::FONT_HERSHEY_SCRIPT_SIMPLEX, 1, Scalar(200, 200, 255), 2);
    putText(temp, lineNumSizeText, temp.size() / 3, cv::FONT_HERSHEY_SCRIPT_SIMPLEX, 1, Scalar(200, 200, 255), 2);
    imshow("EndLineDetect", temp);
}

void dip_process_loop(VideoCapture &capture)
{
    Mat srcFrame, leftFrame, rightFrame;

    capture >> srcFrame;

    resize(srcFrame, srcFrame, srcFrame.size() / 2);

    SplitFrame(srcFrame, leftFrame, rightFrame);

    Mat leftMask, rightMask;

    static int minH = 156;
    static int maxH = 10;
    static int minS = 182;
    static int maxS = 255;
    static int minV = 46;
    static int maxV = 255;

    auto left_thread = thread([&]()
                              {
            medianBlur(leftFrame, leftFrame, 7);
            leftMask = HSVSplitImg(leftFrame, minH, maxH, minS, maxS, minV, maxV);
            erode(leftMask, leftMask, getStructuringElement(MORPH_RECT, Size(3, 3)));
            dilate(leftMask, leftMask, getStructuringElement(MORPH_RECT, Size(3, 5))); });

    auto right_thread = thread([&]()
                               {
            medianBlur(rightFrame, rightFrame, 7);
            rightMask = HSVSplitImg(rightFrame, minH, maxH, minS, maxS, minV, maxV);
            erode(rightMask, rightMask, getStructuringElement(MORPH_RECT, Size(3, 3)));
            dilate(rightMask, rightMask, getStructuringElement(MORPH_RECT, Size(3, 5))); });

    left_thread.join();
    right_thread.join();

    cv::namedWindow("leftMask", cv::WINDOW_AUTOSIZE);
    cv::createTrackbar("minH", "leftMask", &minH, 180);
    cv::createTrackbar("maxH", "leftMask", &maxH, 180);
    cv::createTrackbar("minS", "leftMask", &minS, 255);
    cv::createTrackbar("maxS", "leftMask", &maxS, 255);
    cv::createTrackbar("minV", "leftMask", &minV, 255);
    cv::createTrackbar("maxV", "leftMask", &maxV, 255);

    cv::imshow("leftMask", leftMask);
    cv::imshow("rightMask", rightMask);

    std::vector<Point3i> leftPoints(EdgePointNum);
    std::vector<Point3i> rightPoints(EdgePointNum);
    std::vector<Point3i> middlePoints(EdgePointNum);

    // 更新搜线数据
    SearchLine(leftMask, rightMask, leftFrame, rightFrame, leftPoints, rightPoints, middlePoints);

    // 写入全局变量，供PID端使用
    {
        std::lock_guard<std::mutex> guard(MyMutex);

        kLeftLine = NormlizePoints(leftFrame, leftPoints);
        kRightLine = NormlizePoints(leftFrame, rightPoints);
        kMiddleLine = NormlizePoints(leftFrame, middlePoints);
    };

    char c = waitKey(1);
    switch (c)
    {
    case 'q':
        exit(0);
        break;
    case 'Q':
        exit(0);
        break;
    default:
        break;
    }
}
