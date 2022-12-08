#include "pid_calc.hpp"
#include <atomic>
#include <thread>
#include <mutex>
#include "cv_thread.hpp"

using namespace std;
using namespace cv;

atomic<PID_CALC_STATE> PidCalcState;

// 左到右对应[0,1]
#define EXP_LEFT 0.3
#define EXP_MIDDLE 0.5
#define EXP_RIGHT 0.8
// 最下方0，最上方99
#define LINE_POS 15

const double Kp = 5.5;
const double Ki = 0.0;
const double Kd = 0.25;
const double Vcar = 0.4;
const double LimitDistance = 0.45;
const double magicOmega = 0.5;

bool lineFlag = false;

#define magicTime 1s

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

    cout << "diff:" << diff << "Mode" << mode << '\t';
    cout << middleLine[LINE_POS] << endl;
    // cout << middleLine;
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

void magic_turning(int turn, ros::Publisher &vel_pub)
{
    if (turn = 1)
    {
        pub_vel(0, magicOmega, true, vel_pub);
        this_thread::sleep_for(magicTime);
        pub_vel(0, 0, true, vel_pub);
    }
    if (turn = -1)
    {
        pub_vel(0, -magicOmega, true, vel_pub);
        this_thread::sleep_for(magicTime);
        pub_vel(0, 0, true, vel_pub);
    }
}

void pid_main(ros::Publisher *vel_pub)
{
    PidCalcState = PID_CALC_STATE::Running;
    // dip_main_running = true;
    int RunMode = 3;
    bool isMove = true;
    double diff = 0;
    double omega = 0;
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

            // 读取数据
            {
                lock_guard<std::mutex> lock(MyMutex);
                distance = sender_endlineDistanceNormlized;
                endlines_size = sender_endlines.size();
            }

            // PID 计算
            diff = diffCalc(sendleftLine, sendrightLine, sendmiddleLine, RunMode);
            DIPPID(diff, omega);
            if (endlines_size < 2 && lineFlag == false)
            {
                pub_vel(Vcar, omega, isMove, *vel_pub);
            }
            else
            {
                cout << "\nStop!!!!!!!!!!\n"
                     << endl;
                lineFlag = true;
                pub_vel(0, 0, isMove, *vel_pub);
                loop_rate.sleep();
            }

            // 判断是否找到路口横线

            // cout << "distance: " << distance << endl;

            if ((distance > LimitDistance) && (endlines_size > 3))
            // if (true)
            {
                isMove = false;
                pub_vel(0, 0, true, *vel_pub);
                PidCalcState = PID_CALC_STATE::PillDetecting;
                CvState = CV_STATE::PillDetecting;
                this_thread::sleep_for(0.2s);
            }

            break;
        case PID_CALC_STATE::PillDetecting:
            CvState = CV_STATE::PillDetecting;
            isMove = false;
            // dip_main_running = false;

            double compactness, eccentricity;

            // 读取数据
            {
                lock_guard<std::mutex> lock(MyMutex);
                compactness = sender_compactness;
                eccentricity = sender_eccentricity;
            }

            if (turnDirection == 0)
            {
                if ((compactness < 15) && (compactness > 13) && (eccentricity < 0.5))
                {
                    turnflagCount++;
                }
                else if ((compactness > 15) && (eccentricity > 0.6))
                {
                    turnflagCount--;
                }

                if (turnflagCount > 5)
                {
                    turnDirection = 1;
                }
                if (turnflagCount < -5)
                {
                    turnDirection = -1;
                }
            }
            else
            {
                cout << "turnDirection: " << turnDirection << endl;
                PidCalcState = PID_CALC_STATE::Running;
                CvState = CV_STATE::LineSearching;
                lineFlag = false;
                isMove = true;
                switch (turnDirection)
                {
                case -1:
                    RunMode = 2;
                    for (int i = 0; i < 10; i++)
                    {
                        pub_vel(0, 1.57, true, *vel_pub);
                        loop_rate.sleep();
                    }

                    break;

                case 1:
                    RunMode = 1;
                    for (int i = 0; i < 10; i++)
                    {
                        pub_vel(0, -1.57, true, *vel_pub);
                        loop_rate.sleep();
                    }
                    break;

                default:
                    break;
                }
                // magic_turning(turnDirection, *vel_pub);
                // pub_vel(0, 0, true, *vel_pub);
                // this_thread::sleep_for(0.5s);
                // pub_vel(Vcar, 0, true, *vel_pub);
                // this_thread::sleep_for(3s);
                // pub_vel(0, 0, false, *vel_pub);
                // return;
            }

            break;
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
