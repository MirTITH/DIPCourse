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

//检测矩形
//第一个参数是传入的原始图像，第二是输出的图像。
void findSquares(const Mat &image, Mat &out)
{
    int thresh = 50, N = 5;
    vector<vector<Point>> squares;
    squares.clear();

    Mat src, dst, gray_one, gray;

    src = image.clone();
    out = image.clone();
    gray_one = Mat(src.size(), CV_8U);
    //滤波增强边缘检测
    medianBlur(src, dst, 9);
    // bilateralFilter(src, dst, 25, 25 * 2, 35);

    vector<vector<Point>> contours;
    vector<Vec4i> hierarchy;

    //在图像的每个颜色通道中查找矩形
    for (int c = 0; c < image.channels(); c++)
    {
        int ch[] = {c, 0};

        //通道分离
        mixChannels(&dst, 1, &gray_one, 1, ch, 1);

        // 尝试几个阈值
        for (int l = 0; l < N; l++)
        {
            // 用canny()提取边缘
            if (l == 0)
            {
                //检测边缘
                Canny(gray_one, gray, 5, thresh, 5);
                //膨脹
                dilate(gray, gray, Mat(), Point(-1, -1));
                imshow("dilate", gray);
            }
            else
            {
                gray = gray_one >= (l + 1) * 255 / N;
            }

            // 轮廓查找
            // findContours(gray, contours, RETR_CCOMP, CHAIN_APPROX_SIMPLE);
            findContours(gray, contours, hierarchy, RETR_CCOMP, CHAIN_APPROX_SIMPLE);

            vector<Point> approx;

            // 检测所找到的轮廓
            for (size_t i = 0; i < contours.size(); i++)
            {
                //使用图像轮廓点进行多边形拟合
                approxPolyDP(Mat(contours[i]), approx, arcLength(Mat(contours[i]), true) * 0.02, true);

                //计算轮廓面积后，得到矩形4个顶点
                if (approx.size() == 4 && fabs(contourArea(Mat(approx))) > 1000 && isContourConvex(Mat(approx)))
                {
                    double maxCosine = 0;

                    for (int j = 2; j < 5; j++)
                    {
                        // 求轮廓边缘之间角度的最大余弦
                        double cosine = fabs(angle(approx[j % 4], approx[j - 2], approx[j - 1]));
                        maxCosine = MAX(maxCosine, cosine);
                    }

                    if (maxCosine < 0.3)
                    {
                        squares.push_back(approx);
                    }
                }
            }
        }
    }

    for (size_t i = 0; i < squares.size(); i++)
    {
        const Point *p = &squares[i][0];

        int n = (int)squares[i].size();
        if (p->x > 3 && p->y > 3)
        {
            polylines(out, &p, &n, 1, true, Scalar(0, 255, 0), 3, LINE_AA);
        }
    }
    imshow("dst", out);
}

static double angle(Point pt1, Point pt2, Point pt0)
{
    double dx1 = pt1.x - pt0.x;
    double dy1 = pt1.y - pt0.y;
    double dx2 = pt2.x - pt0.x;
    double dy2 = pt2.y - pt0.y;
    return (dx1 * dx2 + dy1 * dy2) / sqrt((dx1 * dx1 + dy1 * dy1) * (dx2 * dx2 + dy2 * dy2) + 1e-10);
}
