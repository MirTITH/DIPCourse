#include "cv_thread.hpp"
#include "dip_process.hpp"

#define USE_VIDEO 1

using namespace std;
using namespace cv;

atomic<CV_STATE> CvState;
std::mutex MyMutex;

vector<Point3d> kLeftLine(EdgePointNum);
vector<Point3d> kRightLine(EdgePointNum);
vector<Point3d> kMiddleLine(EdgePointNum);

void CvThread()
{
    VideoCapture capture;
#ifdef USE_VIDEO
    capture.open("/home/xy/Documents/2.webm");
#else
    capture.open(1);
#endif

    if (capture.isOpened() == false)
    {
        cout << "CvThread: capture.isOpened() == false" << endl;
        exit(0);
    }

    while (true)
    {
        switch (CvState)
        {
        case CV_STATE::LineSearching:
            dip_process_loop(capture);

#ifdef USE_VIDEO
            // 循环播放
            if (capture.get(cv::CAP_PROP_POS_FRAMES) == capture.get(cv::CAP_PROP_FRAME_COUNT))
            {
                capture.set(cv::CAP_PROP_POS_FRAMES, 0);
            }
#endif

        case CV_STATE::Detecting:
            // TODO
            this_thread::sleep_for(10ms);
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