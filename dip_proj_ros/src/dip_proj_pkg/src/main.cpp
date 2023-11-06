#include <opencv2/opencv.hpp>
#include <vector>
#include "dip_process.hpp"
#include <thread>
#include "pid_calc.hpp"
#include "cv_thread.hpp"

using namespace std;
using namespace cv;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "XY");
    ros::NodeHandle n;
    ros::Publisher vel_pub = n.advertise<geometry_msgs::Twist>("smoother_cmd_vel", 5);

    CvState = CV_STATE::Pause;

    auto cv_thread = thread(CvThread); // 图像识别线程
    cv_thread.detach();

    auto pid_thread = thread(pid_main, &vel_pub); // PID 伺服线程
    pid_thread.join();

    return 0;
}
