#include <opencv2/opencv.hpp>
#include <vector>
#include "dip_process.hpp"
#include <thread>
#include "pill_detect.hpp"
#include "pid_calc.hpp"
#include <signal.h>

using namespace std;
using namespace cv;

// void signal_callback_handler(int signum){
//     cout << "caught: " << signum << endl;
//     exit(0);
// }

int main(int argc, char **argv)
{
    VideoCapture capture;
    capture.open(CameraNumber);
    auto dip_thread = thread(dip_main, &capture);
    pid_main(argc, argv, capture);
    dip_thread.join();
    //pill_detect_main();
    return 0;
}
