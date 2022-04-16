/**
 * @author MENGQY 619597329@qq.com
 * @ref https://dev.intelrealsense.com/docs/code-samples?_ga=2.40280033.219234763.1620224200-92814120.1618399150
 */
#ifndef RS_CAMERA_H
#define RS_CAMERA_H

#include "tools/gl_tools.hpp"
#include <iomanip>
#include <iostream>
#include <librealsense2/rs.hpp>
#include <map>
#include <opencv2/opencv.hpp>

#include <string>
#include <thread>
#include <utility>
#include <vector>

/**
 * rs_camera类是对intel官方历程的二次封装
 * 前面的功能是直接访问相机的函数，例如得到设备名，传感器参数，单独显示某一传感器流等
 * 后面提供了OpenCV接口，要搭配OpenCV请直接使用init_camera_with_OpenCV()和之后的函数
 */

/******重要！！！！******/
// 有空研究一下librealsense/wrapper/opencv这个文件夹，里面有很多OpenCV接口的好东西

class rs_camera
{
public:
    rs_camera() {}

    ~rs_camera() { fs.release(); }

    //得到realsense设备，默认为连接的多台设备中的第一个
    rs2::device get_a_realsense_device();

    //打印设备信息
    void print_device_information(const rs2::device &dev);

    //得到设备名称
    std::string get_device_name(const rs2::device &dev);

    //获取传感器名称
    std::string get_sensor_name(const rs2::sensor &sensor)
    {
        //传感器支持附加信息，例如人类可读的名称
        if (sensor.supports(RS2_CAMERA_INFO_NAME))
            return sensor.get_info(RS2_CAMERA_INFO_NAME);
        else
            return "Unknown Sensor";
    }

    //得到某一传感器
    // index为传感器代号，0,1,2 Stereo Module,RGB Camera,Motion Module
    rs2::sensor get_a_sensor_from_a_device(const rs2::device &dev, int index);

    //更改相机参数
    //测试是曾发现过蓝屏的状况，但是没动代码自己就好了
    //期待大佬发现缘由
    // 0是RGB,1是深度
    void change_sensor_option(const rs2::sensor &sensor, int index);

    //得到设备FOV
    void get_field_of_view(const rs2::stream_profile &stream);

    //得到相机内参
    void get_extrinsics(const rs2::stream_profile &from_stream, const rs2::stream_profile &to_stream)
    {
        //如果您使用的设备/传感器包含多个流，并且已校准
        //然后 SDK 提供了一种获取任意两个流之间转换的方法
        try
        {
            //给定两个流，使用 get_extrinsics_to() 函数获取从流到另一个流的转换
            rs2_extrinsics extrinsics = from_stream.get_extrinsics_to(to_stream);
            std::cout << "Translation Vector : [" << extrinsics.translation[0] << "," << extrinsics.translation[1] << "," << extrinsics.translation[2] << "]\n";
            std::cout << "Rotation Matrix    : [" << extrinsics.rotation[0] << "," << extrinsics.rotation[3] << "," << extrinsics.rotation[6] << "]\n";
            std::cout << "                   : [" << extrinsics.rotation[1] << "," << extrinsics.rotation[4] << "," << extrinsics.rotation[7] << "]\n";
            std::cout << "                   : [" << extrinsics.rotation[2] << "," << extrinsics.rotation[5] << "," << extrinsics.rotation[8] << "]" << std::endl;
        }
        catch (const std::exception &e)
        {
            std::cerr << "Failed to get extrinsics for the given streams. " << e.what() << std::endl;
        }
    }

    //播放实时画面流
    void display_live_stream(rs2::device device, rs2::sensor sensor);

    //开始流式传输
    void start_streaming_a_profile(const rs2::sensor &sensor, const rs2::stream_profile &stream_profile);

    //选择画面流
    rs2::stream_profile choose_a_streaming_profile(const rs2::sensor &sensor);

    /*********************************************************************/
    // OpenCV
    //用OpenCV的方式初始化相机并更改相机参数
    void init_camera_with_opencv();

    //放在while循环里获取帧
    void prepare_frameset()
    {
        frameset_ = pipe.wait_for_frames();
    }

    //得到Mat类型的彩色图像
    cv::Mat get_color_image();
    //得到16U的包含深度数据的未渲染深度图像
    cv::Mat get_depth_image();
    //得到用于展示的已渲染深度图像
    cv::Mat get_colorized_depth_image();
    //得到左右目相机
    cv::Mat get_left_ir_image();
    cv::Mat get_right_ir_image();

private:
    cv::FileStorage fs;
    rs2::frameset frameset_;
    rs2::pipeline_profile profile;
    rs2::pipeline pipe;
    rs2::colorizer c;
    int color_w = 0, color_h = 0;
    int depth_w = 0, depth_h = 0;
    int infrared_w = 0, infrared_h = 0;
    int color_fps = 0, depth_fps = 0, infrared_fps = 0;
};

class frame_viewer
{
public:
    frame_viewer(const std::string &window_title) : _window_title(window_title), _thread(new std::thread(&frame_viewer::run, this)) {}
    ~frame_viewer()
    {
        if (_thread && _thread->joinable())
            _thread->join();
    }
    void operator()(rs2::frame f) { _frames.enqueue(f); }
    void wait()
    {
        // Wait for the windows to close
        if (_thread)
            _thread->join();
    }

private:
    void run()
    {
        window app(640, 480, _window_title.c_str());
        std::string error;
        while (app)
        {
            float view_width = app.width();
            float view_height = app.height();
            if (error.empty())
            {
                rs2::frame frame;
                if (!_frames.poll_for_frame(&frame))
                {
                    frame = _last_frame;
                }
                _last_frame = frame;
                if (frame)
                {
                    try
                    {
                        renderer.render(colorize.process(decimate.process(frame)).as<rs2::video_frame>(), {0, 0, view_width, view_height});
                    }
                    catch (const std::exception &e)
                    {
                        error = e.what();
                    }
                }
            }
            else
            {
                draw_text(int(std::max(0.f, (view_width / 2) - error.length() * 3)), int(view_height / 2), error.c_str());
            }
        }
    }

private:
    texture renderer;
    rs2::colorizer colorize;
    rs2::decimation_filter decimate;
    std::string _window_title;
    rs2::frame_queue _frames;
    rs2::frame _last_frame;
    std::unique_ptr<std::thread> _thread;
};

//在tools/realsense-viewer/realsense-viewer.cpp里有源码
//艹
//费那么大劲写UI
// class Debug
// {
// public:
//     Debug()
//     {
//         fs = cv::FileStorage("../../../param/cameraParm.yaml", cv::FileStorage::READ);
//         // 加载中文字体
//         ImGuiIO &io = ImGui::GetIO();
//         ImFont *font = io.Fonts->AddFontFromFileTTF("../../../third-party/imgui/fonts/HeiTi.ttf", 24.0f, NULL, io.Fonts->GetGlyphRangesChinese());
//     }
//     ~Debug()
//     {
//         // Release resources
//         ImGui_ImplGlfw_Shutdown();
//         glfwTerminate();
//     }

//     void draw_line(window &window_, float x0, float y0, float x1, float y1, int width)
//     {
//         float window_width = window_.width();
//         float window_height = window_.height();
//         glPushAttrib(GL_ENABLE_BIT);
//         glColor3f(1.0f, 0.0f, 0.0f);
//         glLineWidth(5.0f);
//         glBegin(GL_LINES);
//         glVertex3f(x0, y0, 0.0f);
//         glVertex3f(x1, y1, 0.0f);
//         glVertex3f(200.0f, 100.0f, 0.0f);
//         glVertex3f(200.0f, 200.0f, 0.0f);
//         glVertex3f(250.0f, 250.0f, 0.0f);
//         glEnd();
//         glPopAttrib();
//     }

//     void init_device()
//     {
//         config_.enable_stream(RS2_STREAM_DEPTH);
//         config_.enable_stream(RS2_STREAM_COLOR);

//         profile = pipe.start(config_);
//         selected_device = profile.get_device();
//     }
//     void init_sensor()
//     {
//         color_sensor = selected_device.first<rs2::color_sensor>();
//         depth_sensor = selected_device.first<rs2::depth_sensor>();
//         // set_sensor_option(color_sensor, 1);
//     }

//     // void set_sensor_option(const rs2::sensor &sensor, float selected_num = 1) {
//     //     // cv::FileNodeIterator it, it_end;
//     //     // // if (selected_num == 1) {
//     //     // cv::FileNode RGB_camera = fs["RGB_camera"];
//     //     // it = RGB_camera.begin(), it_end = RGB_camera.end();
//     //     cv::FileNode         RGB_camera = fs["RGB_camera"];
//     //     cv::FileNodeIterator it = RGB_camera.begin(), it_end = RGB_camera.end();
//     //     // } else if (selected_num == 2) {
//     //     //     cv::FileNode         depth_camera = fs["depth_camera"];
//     //     //     it = depth_camera.begin(), it_end = depth_camera.end();
//     //     // }
//     //     int idx = 0;

//     //     //使用FileNodeIterator历遍序列（读取）
//     //     for (; it != it_end; it++, idx++) {
//     //         rs2_option option_ = static_cast<rs2_option>((int)(*it)["index"]);
//     //         if (!sensor.supports(option_)) {
//     //             std::cout << option_ << " is not supported" << std::endl;
//     //         }
//     //         rs2::option_range range                   = sensor.get_option_range(option_);
//     //         float             maximum_supported_value = range.max;
//     //         float             minimum_supported_value = range.min;
//     //         float             set_value               = (float)(*it)["value"];
//     //         std::string       set_option              = (std::string)(*it)["option"];
//     //         if (set_value > maximum_supported_value || set_value < minimum_supported_value) {
//     //             std::cerr << "value : " << set_option << " is out of range" << std::endl;
//     //         }
//     //         try {
//     //             sensor.set_option(option_, set_value);

//     //             std::cerr << "param : " << set_option << " has been changed to " << set_value << std::endl;
//     //         } catch (const rs2::error &e) {
//     //             //某些选项只能在相机流式传输时设置，
//     //             //通常硬件可能会失败，因此从 set_option 中捕获异常是一种很好的做法
//     //             std::cerr << "Failed to set option " << option_ << ". (" << e.what() << ")" << std::endl;
//     //         }
//     //     }

//     //     fs.release();
//     // }

//     void slider_init()
//     {
//         ImGui::Text("彩色相机参数");
//         ImGui::Checkbox("自动曝光", &enable_auto_exposure);
//         bool brigtness_ischanged = ImGui::SliderFloat("亮度", &brigtness, -64.0f, 64.0f);
//         bool exposure_ischanged = ImGui::SliderFloat("相机曝光", &exposure, 1.0f, 10000.0f);
//         ImGui::TextWrapped("曝光暂时改不了，请期待后续版本");
//         bool gain_ischanged = ImGui::SliderFloat("相机增益", &gain, 0.0f, 128.0f);
//         bool contrast_ischanged = ImGui::SliderFloat("对比度", &contrast, 0.0f, 100.0f);
//         bool gamma_ischanged = ImGui::SliderFloat("伽马", &gamma, 100.0f, 500.0f);
//         bool hue_ischanged = ImGui::SliderFloat("色调", &hue, -180.0f, 180.0f);
//         bool saturation_ischanged = ImGui::SliderFloat("饱和度", &saturation, 0.0f, 100.0f);
//         bool sharpness_ischanged = ImGui::SliderFloat("锐度", &sharpness, 0.0f, 100.0f);
//         bool white_balance_ischanged = ImGui::SliderFloat("白平衡", &white_balance, 2800.0f, 6500.0f);
//         ImGui::Text("深度相机参数：");
//         ImGui::Checkbox("深度相机自动曝光", &enable_auto_depth_exposure);
//         bool depth_exposure_ischanged = ImGui::SliderFloat("曝光", &Exposure, 1.0f, 16500.0f);
//         bool laserpower_ischanged = ImGui::SliderFloat("激光功率", &Laser_power, 0.0f, 360.0f);

//         if (color_sensor.supports(RS2_OPTION_ENABLE_AUTO_EXPOSURE))
//         {
//             color_sensor.set_option(RS2_OPTION_ENABLE_AUTO_EXPOSURE, enable_auto_exposure);
//         }
//         if (brigtness_ischanged && color_sensor.supports(RS2_OPTION_BRIGHTNESS))
//         {
//             color_sensor.set_option(RS2_OPTION_BRIGHTNESS, brigtness);
//         }
//         if (exposure_ischanged && color_sensor.supports(RS2_OPTION_EXPOSURE))
//         {
//             color_sensor.set_option(RS2_OPTION_EXPOSURE, exposure);
//         }
//         if (contrast_ischanged && color_sensor.supports(RS2_OPTION_CONTRAST))
//         {
//             color_sensor.set_option(RS2_OPTION_CONTRAST, contrast);
//         }
//         if (gain_ischanged && color_sensor.supports(RS2_OPTION_GAIN))
//         {
//             color_sensor.set_option(RS2_OPTION_GAIN, gain);
//         }
//         if (hue_ischanged && color_sensor.supports(RS2_OPTION_HUE))
//         {
//             color_sensor.set_option(RS2_OPTION_HUE, hue);
//         }
//         if (saturation_ischanged && color_sensor.supports(RS2_OPTION_SATURATION))
//         {
//             color_sensor.set_option(RS2_OPTION_SATURATION, saturation);
//         }
//         if (sharpness_ischanged && color_sensor.supports(RS2_OPTION_SHARPNESS))
//         {
//             color_sensor.set_option(RS2_OPTION_SHARPNESS, sharpness);
//         }
//         if (white_balance_ischanged && color_sensor.supports(RS2_OPTION_WHITE_BALANCE))
//         {
//             color_sensor.set_option(RS2_OPTION_WHITE_BALANCE, white_balance);
//         }
//         if (gamma_ischanged && color_sensor.supports(RS2_OPTION_GAMMA))
//         {
//             color_sensor.set_option(RS2_OPTION_GAMMA, gamma);
//         }

//         if (depth_sensor.supports(RS2_OPTION_ENABLE_AUTO_EXPOSURE))
//         {
//             depth_sensor.set_option(RS2_OPTION_ENABLE_AUTO_EXPOSURE, enable_auto_depth_exposure);
//         }
//         if (depth_exposure_ischanged && depth_sensor.supports(RS2_OPTION_EXPOSURE))
//         {
//             depth_sensor.set_option(RS2_OPTION_EXPOSURE, Exposure);
//         }
//         if (laserpower_ischanged && depth_sensor.supports(RS2_OPTION_LASER_POWER))
//         {
//             depth_sensor.set_option(RS2_OPTION_LASER_POWER, Laser_power);
//         }
//     }

//     void show_custom_window()
//     {
//         if (custom_window)
//         {
//             ImGui::Begin("用户窗口", &custom_window); // Pass a pointer to our bool variable (the window will have a closing button that will clear the bool when clicked)
//             ImGui::Text("此窗口可以输出调试信息");
//             ImGui::PushItemWidth(-1);
//             ImGui::PopItemWidth();
//             if (ImGui::IsItemHovered())
//                 ImGui::SetTooltip("TDT realsense 自定义调试窗口");
//             if (ImGui::Button("关闭"))
//             {
//                 custom_window = false;
//             }
//             ImGui::End();
//         }
//     }

//     void show_point_cloud()
//     {
//         if (!color_frame)
//             color_frame = frameset.get_infrared_frame();

//         // Tell pointcloud object to map to this color frame
//         pc.map_to(color_frame);

//         // Generate the pointcloud and texture mappings
//         points = pc.calculate(depth_frame);

//         // Upload the color frame to OpenGL
//         app_state.tex.upload(color_frame);

//         // Draw the pointcloud
//         draw_pointcloud(app_width, app_height, app_state, points);
//     }

//     void init_app()
//     {
//         window app(1280, 720, "T-DT RealSense UI");

//         ImGui_ImplGlfw_Init(app, false);

//         static float f = 0.0f;

//         register_glfw_callbacks(app, app_state);

//         while (app)
//         {
//             double t1 = cv::getTickCount();
//             this->app_width = static_cast<float>(app.width());
//             this->app_height = static_cast<float>(app.height());

//             static const int flags = ImGuiWindowFlags_NoCollapse | ImGuiWindowFlags_NoScrollbar | ImGuiWindowFlags_NoSavedSettings | ImGuiWindowFlags_NoTitleBar | ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoMove;

//             ImGui_ImplGlfw_NewFrame(1);

//             // ImGui::SetNextWindowPos({0,0});
//             ImGui::SetNextWindowSize({app.width(), app.height()});

//             ImGui::Begin("Debug", nullptr, flags);
//             ImGui::SetWindowPos(ImVec2(0, 0), ImGuiSetCond_Always);
//             ImGui::SetWindowSize(ImVec2(left_window_width, app_height), ImGuiSetCond_Always);

//             ImGui::Checkbox("深度画面", &show_depth_stream);
//             ImGui::Checkbox("色彩画面", &show_RGB_stream);
//             ImGui::Checkbox("开启点云", &show_PointCloud);
//             slider_init();
//             ImGui::Checkbox("用户自定义窗口", &custom_window);
//             // show_custom_window();

//             ImGui::Text("用户自定义窗口");
//             ImGui::PushItemWidth(-1);
//             ImGui::PopItemWidth();
//             if (ImGui::IsItemHovered())
//                 ImGui::SetTooltip("此窗口可显示调试信息");
//             int button = 0; bool repeat = false;

//             if (ImGui::IsMouseClicked(button,repeat))
//             {
//                 ImVec2 pos = ImGui::GetMousePos();
//                 std::cout<<"pose: "<<pos.x<<" "<<pos.y<<std::endl;
//             }
//             frameset = pipe.wait_for_frames();
//             color_frame = frameset.get_color_frame();
//             depth_frame = frameset.get_depth_frame();
//             auto colorized_depth = colorizer_.colorize(depth_frame);

//             // ImGui::Text("this is test");
//             // text_box box("text box", ImVec2(50, 100), ImVec2(50, 20));
//             // box.remove_title_bar();
//             // box.show("text box");
//             // glViewport(int(w) / 2, int(h) / 2, int(w) / 2, int(h) / 2);

//             // imgui画线和矩形
//             // ImVec4      colf = ImVec4(1.0f, 1.0f, 0.4f, 1.0f);
//             // const ImU32 col  = ImColor(colf);

//             // //绘制图形
//             // ImDrawList *draw_list = ImGui::GetWindowDrawList();
//             // //画线
//             // // draw_list->AddLine(ImVec2(100, 20), ImVec2(500, 500), col, 3.0f);

//             // //画文字
//             // draw_list->AddText(ImVec2(30, 500), col, "TDT ICRA");
//             //画矩形
//             // draw_list->AddRect(ImVec2(50, 100), ImVec2(500, 500), col, 0.0f, 0, 3.0f);

//             // Render the UI:
//             if (show_PointCloud)
//                 show_point_cloud();
//             ImGui::End();
//             ImGui::Render();
//             rect r{0, 0, 848, 480};

//             if (show_depth_stream)
//                 depth_image.render(colorized_depth, {left_window_width + 40, 0, app_width - left_window_width, app_height}, 1 - alpha);
//             else if (show_RGB_stream)
//                 color_image.render(color_frame, {left_window_width + 40, 0, app_width - left_window_width, app_height}, alpha);
//             // app.show(color_frame, {int(app_width) / 2, int(app_height) / 2, int(app_width) / 2, int(app_height) / 2});
//             // depth_image.render(colorized_depth, { 0, 0, app.width(), app.height() },alpha);
//             // draw_line(app.width() / 2, app.width() / 2, app.height() / 3, app.height() / 3, 2);
//             // draw_line(app.width() / 3, app.width() / 2, app.height() / 3, app.height() / 3, 2);
//             double fps = ((double)cv::getTickCount() - t1) / cv::getTickFrequency();
//             // std::cout << "fps = " << 1.0 / fps << std::endl;
//         }
//     }

//     void calc_rect() {}

// private:
//     float left_window_width = 260;
//     float left_window_height = 0;
//     float app_width, app_height = 0;
//     float alpha = 0.5f; // 透明度系数
//     texture depth_image, color_image;
//     rs2::sensor color_sensor, depth_sensor;
//     rs2::device selected_device;
//     rs2::pipeline_profile profile;
//     //声明用于显示启用流的流率的printer
//     rs2::rates_printer printer;
//     //创建管道以方便地配置和启动相机
//     rs2::pointcloud pc;
//     rs2::points points;
//     rs2::pipeline pipe;
//     rs2::config config_;
//     rs2::colorizer colorizer_;
//     rs2::frame color_frame;
//     rs2::frame depth_frame;
//     // rs2::frame      colorized_depth;
//     rs2::frameset frameset;
//     rs2_error *e = 0;
//     bool show_RGB_stream = false;
//     bool show_depth_stream = false;
//     bool show_PointCloud = false;

//     cv::FileStorage fs;
//     glfw_state app_state;

//     //色彩相机参数
//     float brigtness = 0.f, exposure = 100, gain = 64, contrast = 50.f;
//     float gamma = 300, hue = 0.f, saturation = 64, sharpness = 50.f, white_balance = 4600;
//     //深度相机参数
//     float Exposure = 33000, Gain = 16, Laser_power = 150;
//     bool enable_auto_exposure = false;
//     bool enable_auto_depth_exposure = false;
//     bool custom_window = false;
// };

// static void draw_circle(float xx, float xy, float xz, float yx, float yy, float yz, float radius = 1.1, float3 center = {0.0, 0.0, 0.0}, float intensity = 0.5f)
// {
//     const auto N = 50;
//     glColor3f(intensity, intensity, intensity);
//     glLineWidth(2);
//     glBegin(GL_LINE_STRIP);

//     for (int i = 0; i <= N; i++)
//     {
//         const double theta = (2 * M_PI / N) * i;
//         const auto cost = static_cast<float>(cos(theta));
//         const auto sint = static_cast<float>(sin(theta));
//         glVertex3f(center.x + radius * (xx * cost + yx * sint), center.y + radius * (xy * cost + yy * sint), center.z + radius * (xz * cost + yz * sint));
//     }

//     glEnd();
// }

#endif
