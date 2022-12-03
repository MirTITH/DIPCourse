#include <opencv2/opencv.hpp>
#include <vector>
#include "dip_process.hpp"
#include <thread>

using namespace std;
using namespace cv;

int main(int argc, char **argv)
{
    auto dip_thread = thread(dip_main);
    dip_thread.join();
    return 0;
}
