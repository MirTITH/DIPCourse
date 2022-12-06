#include "color_split.hpp"

using namespace std;
using namespace cv;

Mat HSVSplitImg(const Mat &srcImg, int minH, int maxH, int minS, int maxS, int minV, int maxV)
{
    Mat mask[3], hsvImg;

    cvtColor(srcImg, hsvImg, cv::COLOR_BGR2HSV);

    vector<Mat> Channels;
    split(hsvImg, Channels);
    inRange(Channels[0], minH, maxH, mask[0]);
    inRange(Channels[1], minS, maxS, mask[1]);
    inRange(Channels[2], minV, maxV, mask[2]);

    return mask[0] & mask[1] & mask[2];
}

Mat ShowHSVSplitImg(const Mat &srcImg, int minH, int maxH, int minS, int maxS, int minV, int maxV)
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

Mat ShowHSVSplitImgTrackbar(const Mat &srcImg)
{
    Mat mask[3], hsvImg, result;
    static int maxH = 180;
    static int minH = 0;
    static int maxS = 255;
    static int minS = 0;
    static int maxV = 255;
    static int minV = 0;

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

    imshow("HChannel", resultChannel[0]);
    imshow("SChannel", resultChannel[1]);
    imshow("VChannel", resultChannel[2]);

    srcImg.copyTo(result, (mask[0] & mask[1] & mask[2]));

    imshow("HSVSplitImg", result);

    createTrackbar("maxH", "HSVSplitImg", &maxH, 180);
    createTrackbar("minH", "HSVSplitImg", &minH, 180);
    createTrackbar("maxS", "HSVSplitImg", &maxS, 255);
    createTrackbar("minS", "HSVSplitImg", &minS, 255);
    createTrackbar("maxV", "HSVSplitImg", &maxV, 255);
    createTrackbar("minV", "HSVSplitImg", &minV, 255);

    return mask[0] & mask[1] & mask[2];
}

void ColorEqualize(const Mat &srcImg, Mat &destImg)
{
    // Convert BgR image to YCbCr
    Mat ycrcb;
    cvtColor(srcImg, ycrcb, COLOR_BGR2YCrCb);
    // Split image into channels
    vector<Mat> channels;
    split(ycrcb, channels);
    // Equalize the Y channel only
    equalizeHist(channels[0], channels[0]);
    // Merge the result channels
    merge(channels, ycrcb);
    // Convert color ycrcb to BgR
    cvtColor(ycrcb, destImg, COLOR_YCrCb2BGR);
}
