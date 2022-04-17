#include "RS_Camera.h"
#include <opencv2/opencv.hpp>
using namespace cv;
using namespace std;
#include <time.h>
#include <iostream>
#include <fstream>
#include <chrono>
#include <sys/time.h>
// #define USE_OPENCV
int main()
{
    rs_camera camera;
    // //以下是使用OpenCV的启动方式
#ifdef USE_OPENCV
    camera.init_camera_with_opencv();
    while (1)
    {
        camera.prepare_frameset();
        // cv::Mat color = camera.get_color_image();
        cv::Mat left_image = camera.get_left_ir_image();
        // cv::Mat depth4 = camera.get_colorized_depth_image();
        cv::Mat right_image = camera.get_right_ir_image();
        // cv::Mat depth = camera.get_depth_image();
        // cv::imshow("color",color);
        cv::imshow("left", left_image);
        cv::imshow("right", right_image);
        // cv::imshow("depth", depth);
        // cv::imshow("depth4", depth4);
        char ret = cv::waitKey(1);
    }
#else
    // 以下是另一种，不使用opencv的启动方式
    rs2::device dev = camera.get_a_realsense_device();
    camera.print_device_information(dev);
    rs2::sensor sensor_ = camera.get_a_sensor_from_a_device(dev, 1);
    camera.display_live_stream(dev, sensor_);
#endif
    return 0;
}