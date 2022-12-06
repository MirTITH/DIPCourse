#include "line_detect.hpp"

using namespace cv;
using namespace std;

void DrawPoints(cv::Mat &srcImg, vector<Point3i> points, int radius, const cv::Scalar &color, int thickness)
{
    for (auto point : points)
    {
        auto point2d = Point2i(point.x, point.y);
        if (point.z != 0)
        {
            circle(srcImg, point2d, radius, color, thickness);
        }
        else
        {
            circle(srcImg, point2d, radius, color * 0.2, thickness);
        }
    }
}

/**
 * [0,1]
*/
Point2d NormlizePoint(const Mat &srcImg, Point2i real_point)
{
    Point2d normalized_point;
    normalized_point.x = real_point.x / (srcImg.cols - 1);
    normalized_point.y = real_point.y / (srcImg.rows - 1);
    return normalized_point;
}

Point3d NormlizePoint(const Mat &srcImg, Point3i real_point)
{
    Point3d normalized_point;
    normalized_point.x = real_point.x / (srcImg.cols - 1);
    normalized_point.y = real_point.y / (srcImg.rows - 1);
    normalized_point.z = real_point.z;
    return normalized_point;
}

/**
 * [0,1]
*/
std::vector<Point2d> NormlizePoints(const Mat &srcImg, std::vector<Point2i> real_points)
{
    std::vector<Point2d> normalized_points;
    for(auto real_point : real_points)
    {
        normalized_points.push_back(NormlizePoint(srcImg, real_point));
    }
    return normalized_points;
}

std::vector<Point3d> NormlizePoints(const Mat &srcImg, std::vector<Point3i> real_points)
{
    std::vector<Point3d> normalized_points;
    for(auto real_point : real_points)
    {
        normalized_points.push_back(NormlizePoint(srcImg, real_point));
    }
    return normalized_points;
}

Point2d InverseNormlizePoint(const Mat &srcImg, Point2d normalized_point)
{
    Point2d real_point;
    real_point.x = normalized_point.x * (srcImg.cols - 1);
    real_point.y = normalized_point.y * (srcImg.rows - 1);
    return real_point;
}

/**
 * @brief 检测左边界
 *
 * @return true 检测到
 * @return false 未检测到，同时会给 value 赋值 0
 */
bool GetLeftLinePoint(const Mat &srcImg, Point2i &result_pos, Point2i start_pos)
{
    for (int x = start_pos.x; x >= 0; x--)
    {
        if (srcImg.at<uint8_t>(start_pos.y, x) > 0)
        {
            result_pos.x = x;
            result_pos.y = start_pos.y;
            return true;
        }
    }
    result_pos.x = 0;
    result_pos.y = start_pos.y;
    return false;
}

Point3i GetLeftLinePoint(const Mat &srcImg, Point2i start_pos)
{
    Point2i temp;
    Point3i result_pos;

    bool isDetected = GetLeftLinePoint(srcImg, temp, start_pos);

    result_pos.x = temp.x;
    result_pos.y = temp.y;
    result_pos.z = isDetected;
    return result_pos;
}

/**
 * @brief 检测右边界
 *
 * @return true 检测到
 * @return false 未检测到，同时会给 value 赋值 cols - 1
 */
bool GetRightLinePoint(const Mat &srcImg, Point2i &result_pos, Point2i start_pos)
{
    for (int x = start_pos.x; x < srcImg.cols; x++)
    {
        if (srcImg.at<uint8_t>(start_pos.y, x) > 0)
        {
            result_pos.x = x;
            result_pos.y = start_pos.y;
            return true;
        }
    }
    result_pos.x = srcImg.cols - 1;
    result_pos.y = start_pos.y;
    return false;
}

Point3i GetRightLinePoint(const Mat &srcImg, Point2i start_pos)
{
    Point2i temp;
    Point3i result_pos;

    bool isDetected = GetRightLinePoint(srcImg, temp, start_pos);

    result_pos.x = temp.x;
    result_pos.y = temp.y;
    result_pos.z = isDetected;
    return result_pos;
}

void GetLeftLinePoints(const Mat &srcImg, vector<Point3i> &points, double normlized_start_pos)
{
    // Mat temp;
    // srcImg.copyTo(temp);

    // cvtColor(temp, temp, COLOR_GRAY2BGR);

    int start_pos = InverseNormlizePoint(srcImg, Point2d(normlized_start_pos, 0)).x;
    int delta_start_pos = InverseNormlizePoint(srcImg, Point2d(0.1, 0)).x;
    auto start_point = Point2i(start_pos, 0);

    for (auto &point : points)
    {
        start_point.y = point.y;

        // circle(temp, start_point, 1, Scalar(200, 100, 100), 1);

        point = GetLeftLinePoint(srcImg, start_point);
        start_point.x = point.x + delta_start_pos;

        // circle(temp, Point(point.x, point.y), 2, Scalar(255, 100, 100), 2);
    }
    // imshow("GetLeftLinePoints", temp);
}

void GetRightLinePoints(const Mat &srcImg, vector<Point3i> &points, double normlized_start_pos)
{
    // Mat temp;
    // srcImg.copyTo(temp);

    // cvtColor(temp, temp, COLOR_GRAY2BGR);

    int start_pos = InverseNormlizePoint(srcImg, Point2d(normlized_start_pos, 0)).x;
    int delta_start_pos = InverseNormlizePoint(srcImg, Point2d(0.1, 0)).x;
    auto start_point = Point2i(start_pos, 0);

    for (auto &point : points)
    {
        start_point.y = point.y;

        // circle(temp, start_point, 1, Scalar(200, 100, 100), 1);

        point = GetRightLinePoint(srcImg, start_point);
        start_point.x = point.x - delta_start_pos;

        // circle(temp, Point(point.x, point.y), 2, Scalar(255, 100, 100), 2);
    }
    // imshow("GetRightLinePoints", temp);
}

/**
 * @brief 左闭右开
 *
 * @param points
 * @param start
 * @param end
 */
void LinespaceY(std::vector<cv::Point3i> &points, double start, double end)
{
    double delta = (end - start) / points.size();
    double now_value = start;
    for (auto &point : points)
    {
        point.y = now_value;
        now_value += delta;
    }
}
