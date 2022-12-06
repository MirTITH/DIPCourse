#include "pid_calc.hpp"
using namespace std;
using namespace cv;

//左到右对应[0,1]
#define EXP_LEFT 0.3
#define EXP_MIDDLE 0.5
#define EXP_RIGHT 0.8
//最下方0，最上方99
#define LINE_POS 50

double endlineDistance = 0;
vector<Vec2f> endlines;
vector<Point3d> sendleftLine(EdgePointNum);
vector<Point3d> sendrightLine(EdgePointNum);
vector<Point3d> sendmiddleLine(EdgePointNum);

double diffCalc(vector<Point3d> leftLine,vector<Point3d> rightLine,vector<Point3d> middleLine,int mode)
{
    double diff=0;
    if(mode = 3)
    {
        diff = EXP_MIDDLE - middleLine[LINE_POS].x;
    }
    if(mode = 2)
    {
        diff = EXP_RIGHT - rightLine[LINE_POS].x;
    }
    if(mode = 1)
    {
        diff = EXP_LEFT - leftLine[LINE_POS].x;
    }
    return diff;
}
