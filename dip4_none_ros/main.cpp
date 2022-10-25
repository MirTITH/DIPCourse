#include <opencv2/opencv.hpp>
#include <vector>

using namespace std;
using namespace cv;

typedef struct
{
    int minH;
    int maxH;
    int minS;
    int maxS;
    int minV;
    int maxV;
} HSV_Threshold;

HSV_Threshold yellowOrange = {11, 34, 43, 255, 46, 255};
HSV_Threshold green = {35, 85, 43, 255, 46, 255};
HSV_Threshold blue = {86, 140, 43, 255, 46, 255};
HSV_Threshold red = {141, 180, 43, 255, 46, 255};

int ThisColorNum(Mat srcBGRImg, HSV_Threshold color)
{
    Mat hsvImg, temp;
    cvtColor(srcBGRImg, hsvImg, COLOR_BGR2HSV);
    inRange(hsvImg, Scalar(color.minH, color.minS, color.minV), Scalar(color.maxH, color.maxS, color.maxV), temp);
    return sum(temp)[0] / 255;
}

bool IsInRange(int value, int min, int max)
{
    return (value <= max && value >= min) ? true : false;
}

Mat ShowHSVSplitImg(const Mat srcImg, int minH, int maxH, int minS, int maxS, int minV, int maxV)
{
    Mat mask[3], hsvImg, result;

    cvtColor(srcImg, hsvImg, cv::COLOR_BGR2HSV);

    vector<Mat> Channels;
    split(hsvImg, Channels);
    inRange(Channels[0], minH, maxH, mask[0]);
    inRange(Channels[1], minS, maxS, mask[1]);
    inRange(Channels[2], minV, maxV, mask[2]);

    vector<Mat> resultChannel(3);

    for (int i = 0; i < 3; i++)
    {
        Channels[i].copyTo(resultChannel[i], mask[i]);
    }

    // imshow("HChannel", resultChannel[0]);
    // imshow("SChannel", resultChannel[1]);
    // imshow("VChannel", resultChannel[2]);

    srcImg.copyTo(result, (mask[0] & mask[1] & mask[2]));

    imshow("HSVSplitImg", result);

    return mask[0] & mask[1] & mask[2];
}

void TaskOneAndTwo()
{
    auto srcImg = imread("src/dip4/img/img1.png");
    imshow("Source image", srcImg);

    Mat gaussianImg;
    GaussianBlur(srcImg, gaussianImg, cv::Size(5, 5), 3);
    imshow("gaussianImg", gaussianImg);

    int maxH = 180;
    int minH = 0;
    int maxS = 255;
    int minS = 0;
    int maxV = 255;
    int minV = 0;

    for (;;)
    {
        createTrackbar("maxH", "HSVSplitImg", &maxH, 180);
        createTrackbar("minH", "HSVSplitImg", &minH, 180);
        createTrackbar("maxS", "HSVSplitImg", &maxS, 255);
        createTrackbar("minS", "HSVSplitImg", &minS, 255);
        createTrackbar("maxV", "HSVSplitImg", &maxV, 255);
        createTrackbar("minV", "HSVSplitImg", &minV, 255);

        auto hsvSplitResult = ShowHSVSplitImg(gaussianImg, minH, maxH, minS, maxS, minV, maxV);

        imshow("Binary HSV Split Image", hsvSplitResult);

        // findContours(hsvSplitResult, )

        waitKey(50);
    }
}

void TaskThree()
{
    auto srcImg = imread("src/dip4/img/img1.png");
    // imshow("Source image", srcImg);

    Mat gaussianImg;
    GaussianBlur(srcImg, gaussianImg, cv::Size(5, 5), 3);
    // imshow("gaussianImg", gaussianImg);

    int maxH = 180;
    int minH = 0;
    int maxS = 255;
    int minS = 0;
    int maxV = 255;
    int minV = 0;
    int areaThreshold = 2000;

    for (;;)
    {
        createTrackbar("maxH", "hsvSplitResult", &maxH, 180);
        createTrackbar("minH", "hsvSplitResult", &minH, 180);
        createTrackbar("maxS", "hsvSplitResult", &maxS, 255);
        createTrackbar("minS", "hsvSplitResult", &minS, 255);
        createTrackbar("maxV", "hsvSplitResult", &maxV, 255);
        createTrackbar("minV", "hsvSplitResult", &minV, 255);
        createTrackbar("areaThreshold", "hsvSplitResult", &areaThreshold, 10000);

        Mat hsvImg, mask, hsvSplitResult;

        cvtColor(gaussianImg, hsvImg, COLOR_BGR2HSV);
        inRange(hsvImg, Scalar(minH, minS, minV), Scalar(maxH, maxS, maxV), mask);
        gaussianImg.copyTo(hsvSplitResult, mask);

        auto kernel = getStructuringElement(MORPH_RECT, Size(7, 7));
        erode(mask, mask, kernel);
        dilate(mask, mask, kernel);

        // 边缘检测
        RNG rng(12345);
        vector<vector<Point>> contours;
        vector<Vec4i> hierarchy;
        findContours(mask, contours, RETR_EXTERNAL, CHAIN_APPROX_NONE, Point());

        int maxIndex = 0;
        double maxArea = 0;

        for (int i = 0; i < contours.size(); i++)
        {
            double thisArea = contourArea(contours[i]);

            if (thisArea >= areaThreshold)
            {
                if (thisArea > maxArea)
                {
                    maxArea = thisArea;
                    maxIndex = i;
                }

                Scalar color = Scalar(rng.uniform(0, 256), rng.uniform(0, 256), rng.uniform(0, 256));
                // 拟合椭圆
                RotatedRect rrt = fitEllipse(contours[i]);
                ellipse(hsvSplitResult, rrt, color, 2);
            }
        }
        ellipse(hsvSplitResult, fitEllipse(contours[maxIndex]), Scalar(0, 0, 255), 4);
        imshow("hsvSplitResult", hsvSplitResult);

        Mat contourMask(srcImg.size(), CV_8U, Scalar(0));
        drawContours(contourMask, contours, maxIndex, Scalar(255), FILLED);
        Mat cropedImg;
        gaussianImg.copyTo(cropedImg, contourMask);
        imshow("cropedImg", cropedImg);

        int colorNum[4];

        colorNum[0] = ThisColorNum(cropedImg, green);
        colorNum[1] = ThisColorNum(cropedImg, yellowOrange);
        colorNum[2] = ThisColorNum(cropedImg, red);
        colorNum[3] = ThisColorNum(cropedImg, blue);

        cout << "green:" << colorNum[0] << '\t';
        cout << "yellowOrange:" << colorNum[1] << '\t';
        cout << "red:" << colorNum[2] << '\t';
        cout << "blue:" << colorNum[3] << endl;

        int tempNum = 0;
        int maxColorIndex = 0;

        for (int i = 0; i < 4; i++)
        {
            if (colorNum[i] > tempNum)
            {
                tempNum = colorNum[i];
                maxColorIndex = i;
            }
        }

        switch (maxColorIndex)
        {
        case 0:
            cout << "green!" << endl;
            break;
        case 1:
            cout << "yellowOrange!" << endl;
            break;
        case 2:
            cout << "red!" << endl;
            break;
        case 3:
            cout << "blue!" << endl;
            break;
        default:
            cout << "Unrecognized" << endl;
            break;
        }

        waitKey(50);
    }
}

int main(int argc, char **argv)
{
    // TaskOneAndTwo();
    TaskThree();
    return 0;
}
