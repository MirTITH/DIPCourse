#include "pid_calc.hpp"
#include <atomic>
#include <thread>

using namespace std;
using namespace cv;

//左到右对应[0,1]
#define EXP_LEFT 0.3
#define EXP_MIDDLE 0.5
#define EXP_RIGHT 0.8
//最下方0，最上方99
#define LINE_POS 50

const double Kp = 0.0;
const double Ki = 0.0;
const double Kd = 0.0;
const double Vcar = 0;
const double LimitDistance = 100;
const double magicOmega = 0.5;
#define magicTime 1s 


double endlineDistance = 0;
vector<Vec2f> endlines;
vector<Point3d> sendleftLine(EdgePointNum);
vector<Point3d> sendrightLine(EdgePointNum);
vector<Point3d> sendmiddleLine(EdgePointNum);
atomic_bool dip_main_running;

double diffCalc(vector<Point3d> leftLine, vector<Point3d> rightLine, vector<Point3d> middleLine, int mode)
{
    double diff = 0;
    if (mode = 3)
    {
        diff = EXP_MIDDLE - middleLine[LINE_POS].x;
    }
    if (mode = 2)
    {
        diff = EXP_RIGHT - rightLine[LINE_POS].x;
    }
    if (mode = 1)
    {
        diff = EXP_LEFT - leftLine[LINE_POS].x;
    }
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

void pub_vel(double v, double omega, bool isMove,ros::Publisher &vel_pub)
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

void magic_turning(int turn,ros::Publisher &vel_pub)
{
    if(turn = 1)
    {
        pub_vel(0,magicOmega,true,vel_pub);
        this_thread::sleep_for(magicTime);
        pub_vel(0,0,true,vel_pub);
    }
    if(turn = -1)
    {
        pub_vel(0,-magicOmega,true,vel_pub);
        this_thread::sleep_for(magicTime);
        pub_vel(0,0,true,vel_pub);
    }
}

void pid_main(int argc, char **argv)
{
    dip_main_running = true;
    int RunMode = 3;
    bool isMove = true;
    double diff = 0;
    double omega = 0;
    int turnflag = 0;
    ros::init(argc,argv,"ColorMove");
    ros::NodeHandle n;
    ros::Publisher vel_pub = n.advertise<geometry_msgs::Twist>("cmd_vel", 5);
    while (true)
    {
        diff = diffCalc(sendleftLine, sendrightLine, sendmiddleLine, RunMode);
        DIPPID(diff, omega);
        pub_vel(Vcar,omega,isMove,vel_pub);
        if((endlineDistance < LimitDistance)&&(sizeof(endlines) > 10))
        {
            isMove = false;
            dip_main_running = false;
            this_thread::sleep_for(0.2s);
            turnflag = pill_detect_main();
            magic_turning(turnflag,vel_pub);
            pub_vel(Vcar,0,true,vel_pub);
            this_thread::sleep_for(1s);
            pub_vel(0,0,false,vel_pub);
            return;
        }
    }
}