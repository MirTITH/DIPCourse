#include "pid_calc.hpp"
#include <atomic>
#include <thread>
#include <mutex>
#include "cv_thread.hpp"

using namespace std;
using namespace cv;

atomic<PID_CALC_STATE> PidCalcState;

// 左到右对应[0,1]
#define EXP_LEFT 0.3   // 当车在道路中心时，看到的左线的位置
#define EXP_MIDDLE 0.5 // 当车在道路中心时，看到的中线的位置
#define EXP_RIGHT 0.8  // 当车在道路中心时，看到的右线的位置

// 最下方0，最上方99
#define LINE_POS 15 // 用边线在这个位置的点的偏差做伺服

const double Kp = 5.5;
const double Ki = 0.0;
const double Kd = 0.25;
const double Vcar = 0.2; // 车速

/**
 * @brief 计算车的偏移
 *
 * @param leftLine
 * @param rightLine
 * @param middleLine
 * @param mode 1: 根据左线计算，可以让车在路口左转弯；2: 根据右线计算，可以让车在路口右转弯; 3: 根据中线计算，可以让车直行
 * @return double
 */
double diffCalc(vector<Point3d> &leftLine, vector<Point3d> &rightLine, vector<Point3d> &middleLine, int mode)
{
    std::lock_guard<std::mutex> guard(MyMutex);
    double diff = 0;

    switch (mode)
    {
    case 1:
        diff = EXP_LEFT - leftLine[LINE_POS].x;
        break;
    case 2:
        diff = EXP_RIGHT - rightLine[LINE_POS].x;
        break;
    case 3:
        diff = EXP_MIDDLE - middleLine[LINE_POS].x;
        break;

    default:
        break;
    }

    cout << "diff:" << diff << " Mode" << mode << endl;
    return diff;
}

// omega正为右转，diff正为左偏
static void DIPPID(double diff, double &omega)
{
    static double omegaLast;
    static double diffLast1;
    static double diffLast2;
    omega = Kp * (diff - diffLast1) + Ki * diff + Kd * (diff - 2 * diffLast1 + diffLast2) + omegaLast;
    omegaLast = omega;
    diffLast2 = diffLast1;
    diffLast1 = diff;
}

void pub_vel(double v, double omega, bool isMove, ros::Publisher &vel_pub)
{
    geometry_msgs::Twist vel;
    if (isMove = true)
    {
        vel.linear.x = Vcar;
        vel.linear.y = 0;
        vel.linear.z = 0;
        vel.angular.x = 0;
        vel.angular.y = 0;
        vel.angular.z = omega;
    }
    else
    {
        vel.linear.x = 0;
        vel.linear.y = 0;
        vel.linear.z = 0;
        vel.angular.x = 0;
        vel.angular.y = 0;
        vel.angular.z = 0;
    }
    vel_pub.publish(vel);
}

void pid_main(ros::Publisher *vel_pub)
{
    PidCalcState = PID_CALC_STATE::Running;
    int RunMode = 3;
    bool isMove = true; // 是否让车运动，设为 false 可以停车
    double diff = 0;    // 车与期望方向的偏差
    double omega = 0;   // 车的角速度
    int turnflagCount = 0;
    int turnDirection = 0; // 1右转, -1左转,0 待判断

    double distance = 0;
    int endlines_size = 0;

    ros::Rate loop_rate(10);

    CvState = CV_STATE::LineSearching;

    this_thread::sleep_for(1s);

    while (ros::ok())
    {
        switch (PidCalcState)
        {
        case PID_CALC_STATE::Running:
            CvState = CV_STATE::LineSearching;

            // PID 计算
            diff = diffCalc(kLeftLine, kRightLine, kMiddleLine, RunMode);
            DIPPID(diff, omega);
            pub_vel(Vcar, omega, isMove, *vel_pub);

            // TO DO
            // if(岔道口){
            //     PidCalcState = PID_CALC_STATE::Detecting;
            // }

            break;

        case PID_CALC_STATE::Detecting:
            pub_vel(0, 0, false, *vel_pub); // 停车
            // 检测(TODO)
            // if (turn_left)
            // {
            //     RunMode = 1;
            // }
            // else
            // {
            //     RunMode = 2;
            // }

            // PID 计算
            diff = diffCalc(kLeftLine, kRightLine, kMiddleLine, RunMode);
            DIPPID(diff, omega);
            pub_vel(Vcar, omega, isMove, *vel_pub);

        case PID_CALC_STATE::Stop:
            cout << "pid_main Stopping" << endl;
            return;
            break;

        default:
            break;
        }

        loop_rate.sleep();
    }
    CvState = CV_STATE::Stop;
    cout << "Stopping" << endl;
    this_thread::sleep_for(1s);
    return;
}
