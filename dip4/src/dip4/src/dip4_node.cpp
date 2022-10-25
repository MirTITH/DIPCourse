#include <opencv2/opencv.hpp>

using namespace std;

int main(int argc, char **argv)
{
    auto srcImg = cv::imread("src/dip4/img/img1.png");
    cv::imshow("Source image", srcImg);
    cv::waitKey(0);
    return 0;
}
