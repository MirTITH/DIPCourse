#include "pill_detect.hpp"
#include <thread>

using namespace std;
using namespace cv;

bool IsClosedContour(vector<Vec4i> hierarchys, int index)
{
    if (hierarchys[index][2] != -1)
    {
        return true;
    }
    else
    {
        return false;
    }
}

/**
 * @brief 包括自己
 *
 * @param hierarchy
 * @return int
 */
int GetBrotherNum(vector<Vec4i> hierarchys, int index)
{
    int result = 0;
    int nextIndex = index;

    // 向后查找（包括自己）
    while (nextIndex != -1)
    {
        result++;
        nextIndex = hierarchys[nextIndex][0];
    }

    // 向前查找
    // nextIndex = hierarchys[index][1];
    // while (nextIndex != -1)
    // {
    //     result++;
    //     nextIndex = hierarchys[nextIndex][1];
    // }

    return result;
}

/**
 * @brief 检测闭合的兄弟个数
 *
 * @param hierarchy
 * @return int
 */
int GetClosedBrotherNum(vector<Vec4i> hierarchys, int index)
{
    int result = 0;
    int nextIndex = index;

    // 向后查找（包括自己）
    while (nextIndex != -1)
    {
        if (IsClosedContour(hierarchys, nextIndex))
        {
            result++;
        }

        nextIndex = hierarchys[nextIndex][0];
    }

    // 向前查找
    // nextIndex = hierarchys[index][1];
    // while (nextIndex != -1)
    // {
    //     if (IsClosedContour(hierarchys, nextIndex))
    //     {
    //         result++;
    //     }

    //     nextIndex = hierarchys[nextIndex][1];
    // }

    return result;
}

int GetChildNum(vector<Vec4i> hierarchys, int index)
{
    if (index < 0)
    {
        return 0;
    }
    else if (hierarchys[index][2] < 0)
    {
        return 0;
    }
    else
    {
        return GetBrotherNum(hierarchys, hierarchys[index][2]);
    }
}

void DrawContour(Mat &Img, vector<Point> contour, const Scalar &color)
{
    for (auto point : contour)
    {
        circle(Img, point, 1, color);
    }
}

void DrawBrothers(Mat &Img, int index, vector<Vec4i> hierarchys, vector<vector<Point>> contours, const Scalar &color)
{
    int nextIndex = index;
    while (nextIndex != -1)
    {
        auto nowIndex = nextIndex;
        nextIndex = hierarchys[nowIndex][0];
        DrawContour(Img, contours[nowIndex], color);

        stringstream strstream;
        strstream << hierarchys[nowIndex];
        strstream << ',' << nowIndex;
        putText(Img, strstream.str(), contours[nowIndex][0], FONT_HERSHEY_PLAIN, 1, color);
    }

    // 向前查找
    // nextIndex = hierarchys[index][1];
    // while (nextIndex != -1)
    // {
    //     auto nowIndex = nextIndex;
    //     nextIndex = hierarchys[nowIndex][0];
    //     DrawContour(Img, contours[nowIndex], color);

    //     stringstream strstream;
    //     strstream << hierarchys[nowIndex];
    //     strstream << ',' << nowIndex;
    //     putText(Img, strstream.str(), contours[nowIndex][0], FONT_HERSHEY_PLAIN, 1, color);
    // }

    imshow("contoursImg", Img);
}

void DrawClosedBrothers(Mat &Img, int index, vector<Vec4i> hierarchys, vector<vector<Point>> contours, const Scalar &color)
{
    int nextIndex = index;
    while (nextIndex != -1)
    {
        auto nowIndex = nextIndex;
        nextIndex = hierarchys[nowIndex][0];
        if (IsClosedContour(hierarchys, nowIndex))
            DrawContour(Img, contours[nowIndex], color);

        stringstream strstream;
        strstream << hierarchys[nowIndex];
        strstream << ',' << nowIndex;
        putText(Img, strstream.str(), contours[nowIndex][0], FONT_HERSHEY_PLAIN, 1, color);
    }

    // 向前查找
    // nextIndex = hierarchys[index][1];
    // while (nextIndex != -1)
    // {
    //     auto nowIndex = nextIndex;
    //     nextIndex = hierarchys[nowIndex][0];
    //     if (IsClosedContour(hierarchys, nowIndex))
    //         DrawContour(Img, contours[nowIndex], color);

    //     stringstream strstream;
    //     strstream << hierarchys[nowIndex];
    //     strstream << ',' << nowIndex;
    //     putText(Img, strstream.str(), contours[nowIndex][0], FONT_HERSHEY_PLAIN, 1, color);
    // }

    imshow("contoursImg", Img);
}

vector<vector<Point>> GetClosedBrothers(const vector<Vec4i> hierarchys, const vector<vector<Point>> contours, int index)
{
    vector<vector<Point>> result;
    int nextIndex = index;
    while (nextIndex != -1)
    {
        auto nowIndex = nextIndex;
        nextIndex = hierarchys[nowIndex][0];
        if (IsClosedContour(hierarchys, nowIndex))
            result.push_back(contours[nowIndex]);
    }

    return result;
}

void pill_detect_main()
{
    bool isPause = false;
    // cv::Mat img[2];
    // img[0] = imread("../picture/left.png");
    // img[1] = imread("../picture/right.png");
    VideoCapture cap;
    cap.open(0);
    Mat frame;
    // int index = 0;
    while (true)
    {
        // if (isPause == false)
        // {
        //     index++;
        // }

        // if (index > 1)
        // {
        //     index = 0;
        // }

        // img[index].copyTo(frame);

        if (isPause == false)
        {
            cap >> frame;
        }
        // resize(frame, frame, frame.size() / 2);
        medianBlur(frame, frame, 15);

        erode(frame, frame, getStructuringElement(MORPH_RECT, Size(15, 15)));
        dilate(frame, frame, getStructuringElement(MORPH_RECT, Size(15, 15)));

        Mat canny;
        Canny(frame, canny, 50, 150);

        vector<vector<Point>> contours, pills;
        vector<Vec4i> hierarchys;
        findContours(canny, contours, hierarchys, RETR_TREE, CHAIN_APPROX_NONE);

        imshow("frame", frame);
        imshow("canny", canny);

        Mat contoursImg;
        canny.copyTo(contoursImg);
        cvtColor(contoursImg, contoursImg, COLOR_GRAY2BGR);

        auto color = Scalar(255, 100, 100);

        for (int i = 0; i < hierarchys.size(); i++)
        {
            // Mat temp;
            // contoursImg.copyTo(temp);
            if (GetClosedBrotherNum(hierarchys, i) == 8)
            {
                pills = GetClosedBrothers(hierarchys, contours, i);
                // DrawClosedBrothers(temp, i, hierarchys, contours, Scalar(255, 100, 100));
                // cout << i << ',' << GetClosedBrotherNum(hierarchys, i) << endl;
                // waitKey(1000);
            }
        }

        double compactness = 0;  // 紧致度
        double eccentricity = 0; // 偏心率

        for (auto pill : pills)
        {
            // DrawContour(contoursImg, pill, Scalar(100, 255, 100));
            auto totalArea = contourArea(pill);
            auto totalLength = arcLength(pill, true);
            compactness += (totalLength * totalLength) / totalArea / pills.size();

            auto fitted_ellipse = fitEllipse(pill);
            eccentricity += sqrt(1 - pow(fitted_ellipse.size.aspectRatio(), 2)) / pills.size();

            ellipse(contoursImg, fitted_ellipse, Scalar(255, 100, 100));
        }

        imshow("contoursImg", contoursImg);

        cout << compactness << ' ' << eccentricity << endl;

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
        };
    }
}