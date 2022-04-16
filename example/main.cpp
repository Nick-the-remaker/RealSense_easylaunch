#include "RS_Camera.h"
#include <opencv2/opencv.hpp>
using namespace cv;
using namespace std;
#include <time.h>
#include <iostream>
#include <fstream>
#include <chrono>
#include <sys/time.h>
#define USE_OPENCV
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
// #include <librealsense2/rs.hpp>
// #include <opencv2/opencv.hpp>
// #include <mutex>
// #include <iostream>
// #include <cstring>
// #define PI_FL 3.141592f
// struct short3
// {
//     uint16_t x, y, z;
// };

// struct float3
// {
//     float x, y, z;
//     float3 operator*(float t) { return {x * t, y * t, z * t}; }

//     float3 operator-(float t) { return {x - t, y - t, z - t}; }

//     void operator*=(float t)
//     {
//         x = x * t;
//         y = y * t;
//         z = z * t;
//     }

//     void operator=(float3 other)
//     {
//         x = other.x;
//         y = other.y;
//         z = other.z;
//     }

//     void add(float t1, float t2, float t3)
//     {
//         x += t1;
//         y += t2;
//         z += t3;
//     }
// };

// void render_camera(float3 theta)
// {
//     std::cout << theta.x * 180 / PI_FL << "     "
//               << theta.y * 180 / PI_FL << "     "
//               << (theta.z - PI_FL / 2) * 180 / PI_FL << std::endl
//               << std::endl;
// }

// class rotation_estimator
// {
//     // θ是相机在x、y和z分量中的旋转角度
//     float3 theta;
//     std::mutex theta_mtx;
//     /*α表示陀螺仪和加速度计在θ计算中所占的比例；阿尔法越高，陀螺仪的重量就越大，但太高的数值
//     导致漂移；较低的阿尔法使加速计具有更大的重量，对干扰更为敏感*/
//     float alpha = 0.98f;
//     bool firstGyro = true;
//     bool firstAccel = true;
//     // Keeps the arrival time of previous gyro frame
//     double last_ts_gyro = 0;

// public:
//     // 函数，用于根据来自陀螺的数据计算运动角度的变化
//     void process_gyro(rs2_vector gyro_data, double ts)
//     {
//         if (firstGyro) //在第一次迭代中，仅使用来自加速计的数据来设置相机的初始位置
//         {
//             firstGyro = false;
//             last_ts_gyro = ts;
//             return;
//         }
//         // 保持根据陀螺计算的角度变化
//         float3 gyro_angle;

//         // 使用陀螺仪的数据初始化陀螺仪角度
//         gyro_angle.x = gyro_data.x; // Pitch
//         gyro_angle.y = gyro_data.y; // Yaw
//         gyro_angle.z = gyro_data.z; // Roll

//         // 计算之前和当前陀螺仪帧的到达时间之差
//         double dt_gyro = (ts - last_ts_gyro) / 1000.0;
//         last_ts_gyro = ts;

//         // 角度变化等于陀螺测量值*自上次测量以来经过的时间
//         gyro_angle = gyro_angle * static_cast<float>(dt_gyro);

//         // 将计算出的角度变化应用于当前角度（θ）
//         std::lock_guard<std::mutex> lock(theta_mtx);
//         theta.add(-gyro_angle.z, -gyro_angle.y, gyro_angle.x);
//     }

//     void process_accel(rs2_vector accel_data)
//     {
//         // 保持根据加速计数据计算的角度
//         float3 accel_angle;

//         // 根据加速度计数据计算旋转角度
//         accel_angle.z = atan2(accel_data.y, accel_data.z);
//         accel_angle.x = atan2(accel_data.x, sqrt(accel_data.y * accel_data.y + accel_data.z * accel_data.z));

//         // 如果是第一次迭代，根据加速度计数据设置摄像机的初始姿态（注意Y轴的不同处理方式）
//         std::lock_guard<std::mutex> lock(theta_mtx);
//         if (firstAccel)
//         {
//             firstAccel = false;
//             theta = accel_angle;
//             //由于我们无法使用加速度计数据推断围绕Y轴的角度，因此我们将使用PI作为初始姿势的转换
//             theta.y = PI_FL;
//         }
//         else
//         {
//             // 应用互补滤波器：
//             // -高通滤波器=θ*α：允许短时信号通过，同时过滤掉稳定的信号，用于抵消漂移。
//             // -低通滤波器=加速度*（1-α）：允许通过长期变化，滤除短期波动
//             theta.x = theta.x * alpha + accel_angle.x * (1 - alpha);
//             theta.z = theta.z * alpha + accel_angle.z * (1 - alpha);
//         }
//     }
//     //返回当前旋转角度
//     float3 get_theta()
//     {
//         std::lock_guard<std::mutex> lock(theta_mtx);
//         return theta;
//     }
// };

// bool check_imu_is_supported()
// {
//     bool found_gyro = false;
//     bool found_accel = false;
//     rs2::context ctx;
//     for (auto dev : ctx.query_devices())
//     {
//         //同一设备应支持陀螺仪和加速器
//         found_gyro = false;
//         found_accel = false;
//         for (auto sensor : dev.query_sensors())
//         {
//             for (auto profile : sensor.get_stream_profiles())
//             {
//                 if (profile.stream_type() == RS2_STREAM_GYRO)
//                     found_gyro = true;

//                 if (profile.stream_type() == RS2_STREAM_ACCEL)
//                     found_accel = true;
//             }
//         }
//         if (found_gyro && found_accel)
//             break;
//     }
//     return found_gyro && found_accel;
// }

// int main(int argc, char *argv[])
// try
// {
//     if (!check_imu_is_supported())
//     {
//         std::cerr << "Device supporting IMU not found";
//         return EXIT_FAILURE;
//     }

//     rs2::pipeline pipe;

//     rs2::config cfg;
//     rs2::frameset frameset_;
//     cfg.enable_stream(RS2_STREAM_ACCEL, RS2_FORMAT_MOTION_XYZ32F);
//     cfg.enable_stream(RS2_STREAM_GYRO, RS2_FORMAT_MOTION_XYZ32F);
//     cfg.enable_stream(RS2_STREAM_INFRARED, 1, 848, 480, RS2_FORMAT_Y8, 90);
//     cfg.enable_stream(RS2_STREAM_INFRARED, 2, 848, 480, RS2_FORMAT_Y8, 90);

//     //使用给定的配置启动流媒体；
//     //注意，因为我们只允许IMU流，所以只生成单个帧
//     auto profile = pipe.start();

//     //声明处理摄影机姿势计算的对象
//     rotation_estimator algo;

//     rs2::frame frame;
//     // 将到达的帧投射到运动帧
//     auto motion = frame.as<rs2::motion_frame>();
//     // 如果投射成功且到达的帧来自陀螺流
//     if (motion && motion.get_profile().stream_type() == RS2_STREAM_GYRO && motion.get_profile().format() == RS2_FORMAT_MOTION_XYZ32F)
//     {
//         // 获取当前帧的时间戳
//         double ts = motion.get_timestamp();
//         // 获取陀螺测量值
//         rs2_vector gyro_data = motion.get_motion_data();
//         // 调用函数，根据检索到的度量值计算运动角度
//         algo.process_gyro(gyro_data, ts);
//     }
//     // 如果投射成功且到达的帧来自加速计流
//     if (motion && motion.get_profile().stream_type() == RS2_STREAM_ACCEL && motion.get_profile().format() == RS2_FORMAT_MOTION_XYZ32F)
//     {
//         // 获取加速度计测量值
//         rs2_vector accel_data = motion.get_motion_data();
//         // 调用函数，根据检索到的度量值计算运动角度
//         algo.process_accel(accel_data);
//     }
//     // Main loop
//     while (1)
//     {
//         frameset_ = pipe.wait_for_frames();
//         render_camera(algo.get_theta());
//         rs2::video_frame ir_frame_left = frameset_.get_infrared_frame(1);
//         auto time = ir_frame_left.get_timestamp();
//         cv::Mat left_ir_image(cv::Size(848, 480), CV_8UC1, (void *)ir_frame_left.get_data(), cv::Mat::AUTO_STEP);
//         cv::imshow("left", left_ir_image);
//         cv::waitKey(1);
//     }
//     pipe.stop();

//     return EXIT_SUCCESS;
// }
// catch (const rs2::error &e)
// {
//     std::cerr << "RealSense error calling " << e.get_failed_function() << "(" << e.get_failed_args() << "):\n    " << e.what() << std::endl;
//     return EXIT_FAILURE;
// }
// catch (const std::exception &e)
// {
//     std::cerr << e.what() << std::endl;
//     return EXIT_FAILURE;
// }

// #include <librealsense2/rs.hpp>
// #include <thread>
// #include <iostream>
// #include <opencv2/opencv.hpp>

// using namespace std;
// using namespace cv;

// #define width 640
// #define height 480
// #define fps 30

// void grab_imu()
// {
//     rs2::pipeline imu_pipe;
//     rs2::config imu_cfg;
//     rs2::frameset imu_frames;
//     imu_cfg.enable_stream(RS2_STREAM_ACCEL, RS2_FORMAT_MOTION_XYZ32F);
//     imu_cfg.enable_stream(RS2_STREAM_GYRO, RS2_FORMAT_MOTION_XYZ32F);
//     imu_pipe.start(imu_cfg);
//     int imu_count = 0;
//     while (1)
//     {
//         imu_frames = imu_pipe.wait_for_frames(); //等待所有配置的流生成框架
//         // Get imu data
//         if (rs2::motion_frame accel_frame = imu_frames.first_or_default(RS2_STREAM_ACCEL))
//         {
//             rs2_vector accel_sample = accel_frame.get_motion_data();

//             // std::cout << "Accel:" << accel_sample.x << ", " << accel_sample.y << ", " << accel_sample.z << std::endl;
//         }
//         if (rs2::motion_frame gyro_frame = imu_frames.first_or_default(RS2_STREAM_GYRO))
//         {
//             rs2_vector gyro_sample = gyro_frame.get_motion_data();
//             // std::cout << "Gyro:" << gyro_sample.x << ", " << gyro_sample.y << ", " << gyro_sample.z << std::endl;
//         }
//         imu_count++;
//         std::cout << "imu_count = " << imu_count << std::endl;
//     }
// }

// int main(int argc, char **argv)
// try
// {
//     int frame_count = 0;
//     // judge whether devices is exist or not
//     rs2::context ctx;
//     auto list = ctx.query_devices(); // Get a snapshot of currently connected devices
//     if (list.size() == 0)
//         throw std::runtime_error("No device detected. Is it plugged in?");
//     rs2::device dev = list.front();

//     rs2::frameset frames;
//     // Contruct a pipeline which abstracts the device
//     rs2::pipeline pipe;
//     rs2::config cfg; //创建一个以非默认配置的配置用来配置管道
//     // Add desired streams to configuration
//     cfg.enable_stream(RS2_STREAM_COLOR, width, height, RS2_FORMAT_BGR8, fps);
//     // 向配置添加所需的流
//     cfg.enable_stream(RS2_STREAM_DEPTH, width, height, RS2_FORMAT_Z16, fps);
//     cfg.enable_stream(RS2_STREAM_INFRARED, 1, width, height, RS2_FORMAT_Y8, fps);
//     cfg.enable_stream(RS2_STREAM_INFRARED, 2, width, height, RS2_FORMAT_Y8, fps);

//     // get depth scale
//     // float depth_scale = get_depth_scale(profile.get_device());
//     // start stream
//     pipe.start(cfg); //指示管道使用所请求的配置启动流

//     std::thread imu_thread(grab_imu);
//     while (1)
//     {
//         frames = pipe.wait_for_frames(); //等待所有配置的流生成框架

//         // Align to depth
//         rs2::align align_to_depth(RS2_STREAM_DEPTH);
//         frames = align_to_depth.process(frames);

//         // Get imu data
//         if (rs2::motion_frame accel_frame = frames.first_or_default(RS2_STREAM_ACCEL))
//         {
//             rs2_vector accel_sample = accel_frame.get_motion_data();
//             std::cout << "Accel:" << accel_sample.x << ", " << accel_sample.y << ", " << accel_sample.z << std::endl;
//         }
//         if (rs2::motion_frame gyro_frame = frames.first_or_default(RS2_STREAM_GYRO))
//         {
//             rs2_vector gyro_sample = gyro_frame.get_motion_data();
//             std::cout << "Gyro:" << gyro_sample.x << ", " << gyro_sample.y << ", " << gyro_sample.z << std::endl;
//         }
//         // Get each frame
//         rs2::frame color_frame = frames.get_color_frame();
//         rs2::frame depth_frame = frames.get_depth_frame();
//         rs2::video_frame ir_frame_left = frames.get_infrared_frame(1);
//         rs2::video_frame ir_frame_right = frames.get_infrared_frame(2);

//         // Creating OpenCV Matrix from a color image
//         Mat color(Size(width, height), CV_8UC3, (void *)color_frame.get_data(), Mat::AUTO_STEP);
//         Mat pic_right(Size(width, height), CV_8UC1, (void *)ir_frame_right.get_data());
//         Mat pic_left(Size(width, height), CV_8UC1, (void *)ir_frame_left.get_data());
//         Mat pic_depth(Size(width, height), CV_16U, (void *)depth_frame.get_data(), Mat::AUTO_STEP);

//         // Display in a GUI
//         namedWindow("Display Image", WINDOW_AUTOSIZE);
//         imshow("Display Image", color);
//         waitKey(1);
//         imshow("Display depth", pic_depth * 15);
//         waitKey(1);
//         imshow("Display pic_left", pic_left);
//         waitKey(1);
//         imshow("Display pic_right", pic_right);
//         waitKey(1);
//         frame_count++;
//         std::cout<<"frame_count = "<<frame_count<<std::endl;
//     }
//     return 0;
// }

// // error
// catch (const rs2::error &e)
// {
//     std::cerr << "RealSense error calling " << e.get_failed_function() << "(" << e.get_failed_args() << "):\n    " << e.what() << std::endl;
//     return EXIT_FAILURE;
// }
// catch (const std::exception &e)
// {
//     std::cerr << e.what() << std::endl;
//     return EXIT_FAILURE;
// }

// #include <librealsense2/rs.hpp> // Include RealSense Cross Platform API
// // #include "example.hpp"          // Include short list of convenience functions for rendering

// #include <opencv2/opencv.hpp>

// int main(int argc, char * argv[])
// {
// 	int width = 848;
// 	int height = 480;
// 	int fps = 90;
// 	rs2::config config;
//     rs2::config IRconfig;
//     config.enable_stream(RS2_STREAM_COLOR, width, height, RS2_FORMAT_BGR8, 30);
//     // config.enable_stream(RS2_STREAM_DEPTH, width, height, RS2_FORMAT_Z16, 30);
// 	IRconfig.enable_stream(RS2_STREAM_INFRARED, 1, width, height, RS2_FORMAT_Y8, fps);
// 	IRconfig.enable_stream(RS2_STREAM_INFRARED, 2, width, height, RS2_FORMAT_Y8, fps);

// 	// start pipeline
// 	rs2::pipeline pipeline;
// 	rs2::pipeline_profile pipeline_profile = pipeline.start(IRconfig);

// 	rs2::device selected_device = pipeline_profile.get_device();
// 	auto depth_sensor = selected_device.first<rs2::depth_sensor>();

// 	if (depth_sensor.supports(RS2_OPTION_EMITTER_ENABLED))
// 	{
// 		// depth_sensor.set_option(RS2_OPTION_EMITTER_ENABLED, 1.f); // Enable emitter
// 		depth_sensor.set_option(RS2_OPTION_EMITTER_ENABLED, 0.f); // Disable emitter
// 	}

// 	auto ir1_stream = pipeline_profile.get_stream(RS2_STREAM_INFRARED, 1);
// 	auto ir2_stream = pipeline_profile.get_stream(RS2_STREAM_INFRARED, 2);

// 	rs2_intrinsics intrinsics_1 = ir1_stream.as<rs2::video_stream_profile>().get_intrinsics();
// 	rs2_intrinsics intrinsics_2 = ir2_stream.as<rs2::video_stream_profile>().get_intrinsics();

// 	std::cout<<intrinsics_1.fx<<std::endl;

// 	rs2_extrinsics e = ir1_stream.get_extrinsics_to(ir2_stream);
// 	auto baseline = e.translation[0];

// 	int count = 0;
// 	while (1) // Application still alive?
// 	{
// 		// wait for frames and get frameset
// 		rs2::frameset frameset = pipeline.wait_for_frames();

// 		// get single infrared frame from frameset
// 		//rs2::video_frame ir_frame = frameset.get_infrared_frame();

// 		// get left and right infrared frames from frameset
// 		rs2::video_frame ir_frame_left = frameset.get_infrared_frame(1);
// 		rs2::video_frame ir_frame_right = frameset.get_infrared_frame(2);

// 		cv::Mat dMat_left = cv::Mat(cv::Size(width, height), CV_8UC1, (void*)ir_frame_left.get_data());
// 		cv::Mat dMat_right = cv::Mat(cv::Size(width, height), CV_8UC1, (void*)ir_frame_right.get_data());

// 		cv::imshow("img_l", dMat_left);
// 		cv::imshow("img_r", dMat_right);
// 		char c = cv::waitKey(1);
// 		count++;
// 	}

// 	return EXIT_SUCCESS;
// }
