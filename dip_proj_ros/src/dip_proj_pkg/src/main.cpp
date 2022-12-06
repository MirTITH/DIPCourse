#include <opencv2/opencv.hpp>
#include <vector>
#include "dip_process.hpp"
#include <thread>
#include "pill_detect.hpp"

using namespace std;
using namespace cv;

int main(int argc, char **argv)
{
    // auto dip_thread = thread(dip_main);
    // dip_thread.join();
    dip_main();
    //pill_detect_main();
    return 0;
}
