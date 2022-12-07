#include <opencv2/opencv.hpp>
#include <vector>
#include "dip_process.hpp"
#include <thread>
#include "pill_detect.hpp"
#include "pid_calc.hpp"
#include "cv_thread.hpp"

using namespace std;
using namespace cv;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "XY_SCH");
    ros::NodeHandle n;
    ros::Publisher vel_pub = n.advertise<geometry_msgs::Twist>("smoother_cmd_vel", 5);

    CvState = CV_STATE::Pause;

    auto cv_thread = thread(CvThread);
    cv_thread.detach();

    auto pid_thread = thread(pid_main, &vel_pub);
    pid_thread.join();

    return 0;
}
