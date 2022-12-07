#include "cv_thread.hpp"
#include "dip_process.hpp"
#include "pill_detect.hpp"

using namespace std;
using namespace cv;

atomic<CV_STATE> CvState;
std::mutex MyMutex;

double sender_endlineDistanceNormlized = 0;
vector<Vec2f> sender_endlines;
vector<Point3d> sendleftLine(EdgePointNum);
vector<Point3d> sendrightLine(EdgePointNum);
vector<Point3d> sendmiddleLine(EdgePointNum);
// atomic_bool dip_main_running;
double sender_compactness = 0;
double sender_eccentricity = 0;

void CvThread()
{
    VideoCapture capture;
    // capture.open("/home/xy/Documents/DIPCourse/dip_proj_ros/src/dip_proj_pkg/video/1.webm");
    capture.open(1);

    if (capture.isOpened() == false)
    {
        cout << "CvThread: capture.isOpened() == false" << endl;
        exit(0);
    }

    double compactness = 0;
    double eccentricity = 0;

    while (true)
    {
        switch (CvState)
        {
        case CV_STATE::LineSearching:
            // cout << "CvThread LineSearching" << endl;
            dip_process_loop(capture);
            break;
        case CV_STATE::PillDetecting:
            // cout << "CvThread PillDetecting" << endl;
            pill_detect_loop(capture, compactness, eccentricity);

            {
                lock_guard<mutex> lock(MyMutex);
                sender_compactness = compactness;
                sender_eccentricity = eccentricity;
            }
            break;
        case CV_STATE::Pause:
            cout << "CvThread Pausing" << endl;
            this_thread::sleep_for(10ms);
            break;

        case CV_STATE::Stop:
            cout << "CvThread Stopped" << endl;
            return;
            break;
        default:
            cout << "CvThread:CvState undefined" << endl;
            this_thread::sleep_for(10ms);
            break;
        }
    }
}
