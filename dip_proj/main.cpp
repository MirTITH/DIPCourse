#include <opencv2/opencv.hpp>
#include <vector>
#include "color_split.hpp"
#include "line_detect.hpp"

using namespace std;
using namespace cv;

#define EdgePointNum 100

void SplitFrame(const Mat &srcFrame, Mat &leftFrame, Mat &rightFrame)
{
    auto width = srcFrame.cols;
    auto height = srcFrame.rows;

    Rect leftRect(0, 0, width / 2, height);
    Rect rightRect(width / 2, 0, width / 2, height);

    leftFrame = srcFrame(leftRect);
    rightFrame = srcFrame(rightRect);
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

int main(int argc, char **argv)
{
    VideoCapture capture("../video/1.webm");
    Mat frame;
    bool isPause = false;

    if (!capture.isOpened())
    {
        cout << "Cound not read video" << endl;
    }

    Mat srcFrame, leftFrame, rightFrame;

    while (true)
    {
        // 实现循环播放
        if (capture.get(cv::CAP_PROP_POS_FRAMES) == 600)
        {
            capture.set(cv::CAP_PROP_POS_FRAMES, 0);
        }

        if (!isPause)
        {
            capture >> frame;
            resize(frame, frame, Size(frame.cols / 3, frame.rows / 3));
        }

        frame.copyTo(srcFrame);

        SplitFrame(srcFrame, leftFrame, rightFrame);

        medianBlur(leftFrame, leftFrame, 7);
        medianBlur(rightFrame, rightFrame, 7);

        auto leftMask = HSVSplitImg(leftFrame, 79, 151, 64, 241, 25, 217);
        auto rightMask = HSVSplitImg(rightFrame, 79, 151, 64, 241, 25, 217);

        erode(leftMask, leftMask, getStructuringElement(MORPH_RECT, Size(3, 3)));
        erode(rightMask, rightMask, getStructuringElement(MORPH_RECT, Size(3, 3)));
        dilate(leftMask, leftMask, getStructuringElement(MORPH_RECT, Size(3, 5)));
        dilate(rightMask, rightMask, getStructuringElement(MORPH_RECT, Size(3, 5)));

        imshow("leftMask", leftMask);
        imshow("rightMask", rightMask);

        std::vector<Point3i> leftPoints(EdgePointNum), rightPoints(EdgePointNum), middlePoints(EdgePointNum);

        LinespaceY(leftPoints, leftMask.rows - 1, 0);
        LinespaceY(rightPoints, rightMask.rows - 1, 0);

        GetLeftLinePoints(leftMask, leftPoints, 0.5);
        GetRightLinePoints(rightMask, rightPoints, 0.5);

        // DrawPoints(leftFrame, leftPoints, 2, Scalar(255, 100, 100), 2);
        // DrawPoints(leftFrame, rightPoints, 2, Scalar(100, 100, 255), 2);

        for (int i = 0; i < EdgePointNum; i++)
        {
            if (leftPoints[i].x < rightPoints[i].x)
            {
                middlePoints[i] = (leftPoints[i] + rightPoints[i]) / 2;
            }
            else
            {
                middlePoints[i] = (leftPoints[i] + rightPoints[i]) / 2;
            }
        }

        DrawPoints(leftFrame, middlePoints, 2, Scalar(100, 255, 100), 2);

        imshow("leftPoints", leftFrame);

        // char c = waitKey(1000 / capture.get(cv::CAP_PROP_FPS));
        char c = waitKey(1);
        switch (c)
        {
        case 'q':
            exit(0);
            break;
        case 'p':
            isPause = !isPause;
        default:
            break;
        }
    }
    return 0;
}
