#include "RS_Camera.h"

inline uint32_t get_user_selection(const std::string &prompt_message)
{
    std::cout << "\n"
              << prompt_message;
    uint32_t input;
    std::cin >> input;
    std::cout << std::endl;
    return input;
}

rs2::device rs_camera::get_a_realsense_device()
{
    //首先，创建一个 rs2::context。
    //上下文表示当前平台相对于连接的设备
    rs2::context ctx;

    //使用上下文我们可以在设备列表中获取所有连接的设备
    rs2::device_list devices = ctx.query_devices();

    rs2::device selected_device;
    if (devices.size() == 0)
    {
        std::cerr << "No device connected, please connect a RealSense device" << std::endl;

        //帮助等待设备连接的样板代码
        // SDK 提供 rs2::device_hub 类
        rs2::device_hub device_hub(ctx);

        //使用 device_hub 我们可以阻止程序直到设备连接
        selected_device = device_hub.wait_for_device();
    }
    else
    {
        std::cout << "Found the following devices:\n"
                  << std::endl;

        // device_list 是一个“惰性”的设备容器，它允许
        //设备列表提供了2种迭代方式
        //第一种方法是使用迭代器（在这种情况下隐藏在基于范围的 for 循环中）
        int index = 0;
        for (rs2::device device : devices)
        {
            std::cout << index++ << " : " << get_device_name(device) << std::endl;
        }
        //更新选中的设备
        selected_device = devices[0];
        std::cout << "default selected devices[0]\n"
                  << std::endl;
    }
    return selected_device;
}

void rs_camera::print_device_information(const rs2::device &dev)
{
    //每个设备提供一些关于它自己的信息
    //不同类型的可用信息使用“RS2_CAMERA_INFO_*”枚举表示

    std::cout << "Device information: " << std::endl;
    //下面的代码展示了如何枚举所有的 RS2_CAMERA_INFO
    //请注意，SDK 中的所有枚举类型都以零值开始并以“*_COUNT”值结束
    for (int i = 0; i < static_cast<int>(RS2_CAMERA_INFO_COUNT); i++)
    {
        rs2_camera_info info_type = static_cast<rs2_camera_info>(i);
        // SDK 枚举类型可以流式传输以获取表示它们的字符串
        std::cout << "  " << std::left << std::setw(20) << info_type << " : ";

        //设备可能不支持所有类型的 RS2_CAMERA_INFO。
        //为了防止从“get_info”方法抛出异常，我们首先检查设备是否支持这种类型的信息
        if (dev.supports(info_type))
            std::cout << dev.get_info(info_type) << std::endl;
        else
            std::cout << "N/A" << std::endl;
    }
}

std::string rs_camera::get_device_name(const rs2::device &dev)
{
    std::string name = "Unknown Device";
    if (dev.supports(RS2_CAMERA_INFO_NAME))
        name = dev.get_info(RS2_CAMERA_INFO_NAME);

    //设备的序列号：
    std::string sn = "########";
    if (dev.supports(RS2_CAMERA_INFO_SERIAL_NUMBER))
        sn = std::string("#") + dev.get_info(RS2_CAMERA_INFO_SERIAL_NUMBER);

    return name + " " + sn;
}

rs2::sensor rs_camera::get_a_sensor_from_a_device(const rs2::device &dev, int index)
{
    // index: 0,1,2 Stereo Module,RGB Camera,Motion Module
    // 彩色图像选择RGBA8格式
    std::vector<rs2::sensor> sensors = dev.query_sensors();
    if (index >= sensors.size())
    {
        throw std::out_of_range("Selected sensor index is out of range");
    }
    return sensors[index];
}

void rs_camera::change_sensor_option(const rs2::sensor &sensor, int index)
{
    cv::FileStorage read("../config/cameraParm.yaml", cv::FileStorage::READ);
    //传感器通常有几个选项来控制它们的属性,如曝光度、亮度
    cv::FileNodeIterator it, it_end;
    if (index == 1)
    {
        cv::FileNode RGB_camera = read["RGB_camera"];
        it = RGB_camera.begin(), it_end = RGB_camera.end();
        std::cout << "RGB相机更改参数：" << std::endl;
    }
    else if (index == 2)
    {
        cv::FileNode depth_camera = read["DEPTH_camera"];
        it = depth_camera.begin(), it_end = depth_camera.end();
        std::cout << "DEPTH相机更改参数：" << std::endl;
    }
    //定义it
    int idx = 0;
    //使用FileNodeIterator历遍序列（读取）
    for (; it != it_end; it++, idx++)
    {
        rs2_option option_ = static_cast<rs2_option>((int)(*it)["index"]);
        if (!sensor.supports(option_))
        {
            std::cout << option_ << " is not supported" << std::endl;
        }
        rs2::option_range range = sensor.get_option_range(option_);
        float maximum_supported_value = range.max;
        float minimum_supported_value = range.min;
        float set_value = (float)(*it)["value"];
        std::string set_option = (std::string)(*it)["option"];
        if (set_value > maximum_supported_value || set_value < minimum_supported_value)
        {
            std::cerr << "value : " << option_ << " is out of range" << std::endl;
        }
        try
        {
            sensor.set_option(option_, set_value);
            std::cout << "param : " << option_ << " has been changed to " << set_value << std::endl;
        }
        catch (const rs2::error &e)
        {
            //某些选项只能在相机流式传输时设置，
            //通常硬件可能会失败，因此从 set_option 中捕获异常是一种很好的做法
            std::cerr << "Failed to set option " << option_ << ". (" << e.what() << ")" << std::endl;
        }
    }
    read.release();
}

float get_depth_units(const rs2::sensor &sensor)
{
    //深度流包含由具有深度信息的像素组成的图像。
    //每个像素的值是与相机的距离，以某些距离单位表示。
    //要获得以米为单位的距离，每个像素的值应该乘以传感器的深度刻度
    //这是获取“深度”传感器的比例值的方法：
    if (rs2::depth_sensor dpt_sensor = sensor.as<rs2::depth_sensor>())
    {
        float scale = dpt_sensor.get_depth_scale();
        std::cout << "Scale factor for depth sensor is: " << scale << std::endl;
        return scale;
    }
    else
        throw std::runtime_error("Given sensor is not a depth sensor");
}

void rs_camera::get_field_of_view(const rs2::stream_profile &stream)
{
    if (auto video_stream = stream.as<rs2::video_stream_profile>())
    {
        try
        {
            //如果流确实是视频流，我们现在可以简单地调用 get_intrinsics()
            rs2_intrinsics intrinsics = video_stream.get_intrinsics();

            auto principal_point = std::make_pair(intrinsics.ppx, intrinsics.ppy);
            auto focal_length = std::make_pair(intrinsics.fx, intrinsics.fy);
            rs2_distortion model = intrinsics.model;

            std::cout << "Principal Point         : " << principal_point.first << ", " << principal_point.second << std::endl;
            std::cout << "Focal Length            : " << focal_length.first << ", " << focal_length.second << std::endl;
            std::cout << "Distortion Model        : " << model << std::endl;
            std::cout << "Camera Matrix : [" << focal_length.first << ", 0.0, " << principal_point.first << ", 0.0, " << focal_length.second << ", " << principal_point.second << ", 0.0, 0.0, 1]" << std::endl;
            std::cout << "Distortion Coefficients : [" << intrinsics.coeffs[0] << "," << intrinsics.coeffs[1] << "," << intrinsics.coeffs[2] << "," << intrinsics.coeffs[3] << "," << intrinsics.coeffs[4] << "]" << std::endl;
        }
        catch (const std::exception &e)
        {
            std::cerr << "Failed to get intrinsics for the given stream. " << e.what() << std::endl;
        }
    }
    else if (auto motion_stream = stream.as<rs2::motion_stream_profile>())
    {
        try
        {
            //如果流确实是运动流，我们现在可以简单地调用 get_motion_intrinsics()
            rs2_motion_device_intrinsic intrinsics = motion_stream.get_motion_intrinsics();

            std::cout << " Scale X      cross axis      cross axis  Bias X \n";
            std::cout << " cross axis    Scale Y        cross axis  Bias Y  \n";
            std::cout << " cross axis    cross axis     Scale Z     Bias Z  \n";
            for (int i = 0; i < 3; i++)
            {
                for (int j = 0; j < 4; j++)
                {
                    std::cout << intrinsics.data[i][j] << "    ";
                }
                std::cout << "\n";
            }

            std::cout << "Variance of noise for X, Y, Z axis \n";
            for (int i = 0; i < 3; i++)
                std::cout << intrinsics.noise_variances[i] << " ";
            std::cout << "\n";

            std::cout << "Variance of bias for X, Y, Z axis \n";
            for (int i = 0; i < 3; i++)
                std::cout << intrinsics.bias_variances[i] << " ";
            std::cout << "\n";
        }
        catch (const std::exception &e)
        {
            std::cerr << "Failed to get intrinsics for the given stream. " << e.what() << std::endl;
        }
    }
    else
    {
        std::cerr << "Given stream profile has no intrinsics data" << std::endl;
    }
}

void rs_camera::start_streaming_a_profile(const rs2::sensor &sensor, const rs2::stream_profile &stream_profile)
{
    sensor.open(stream_profile);

    std::ostringstream oss;
    oss << "Displaying profile " << stream_profile.stream_name();
    frame_viewer display(oss.str());
    sensor.start([&](rs2::frame f)
                 { display(f); });
    //此时，帧将异步到达回调处理程序
    //该线程将继续并行运行。
    //为了防止这个线程返回，我们使用 helper wait() 函数阻止它
    std::cout << "Streaming profile: " << stream_profile.stream_name() << ". Close display window to continue..." << std::endl;
    display.wait();

    //要停止流式传输，我们只需要调用传感器的停止方法
    //从 stop() 调用返回后，将不会从该传感器到达任何帧
    sensor.stop();

    //要完成停止操作，并释放设备的访问权限，我们需要为每个传感器调用 close()
    sensor.close();
}

rs2::stream_profile rs_camera::choose_a_streaming_profile(const rs2::sensor &sensor)
{
    //传感器是能够传输一种或多种类型数据的对象。
    //例如：
    //*具有左右红外流的立体传感器
    //创建深度图像流
    //*带有加速度计和陀螺仪的运动传感器
    //提供运动信息流

    // 使用传感器，我们可以获得它的所有流配置文件
    std::vector<rs2::stream_profile> stream_profiles = sensor.get_stream_profiles();

    //通常一个传感器提供一个或多个流，这些流可以通过它们的stream_type 和stream_index 来识别
    //这些流中的每一个都可以有多个配置文件（例如 FHD/HHD/VGA/QVGA 分辨率，或 90/60/30 fps 等。）
    //以下代码显示如何查看传感器的流配置文件，并按流对配置文件进行分组。
    std::map<std::pair<rs2_stream, int>, int> unique_streams;
    for (auto &&sp : stream_profiles)
    {
        unique_streams[std::make_pair(sp.stream_type(), sp.stream_index())]++;
    }
    std::cout << "Sensor consists of " << unique_streams.size() << " streams: " << std::endl;
    for (size_t i = 0; i < unique_streams.size(); i++)
    {
        auto it = unique_streams.begin();
        std::advance(it, i);
        std::cout << "  - " << it->first.first << " #" << it->first.second << std::endl;
    }

    //接下来，我们查看所有流配置文件并打印每个配置文件的详细信息        std::cout << "Sensor provides the following stream profiles:" << std::endl;
    int profile_num = 0;
    for (rs2::stream_profile stream_profile : stream_profiles)
    {
        rs2_stream stream_data_type = stream_profile.stream_type();
        int stream_index = stream_profile.stream_index();

        // 2) 每个流都有一个用户友好的名称。
        //流的名称不承诺是唯一的，
        //而不是流的人类可读描述
        std::string stream_name = stream_profile.stream_name();

        // 3) 系统中的每个流，源自相同的
        // rs2::context, 有一个唯一标识符
        //此标识符在所有流中都是唯一的，无论流类型如何。
        int unique_stream_id = stream_profile.unique_id(); //唯一标识符可用于比较两个流
        std::cout << std::setw(3) << profile_num << ": " << stream_data_type << " #" << stream_index;

        //如前所述，流是一种抽象。
        //为了获取特定类型的附加数据
        //流，提供了“Is”和“As”的机制：
        if (stream_profile.is<rs2::video_stream_profile>()) //"Is" 将测试测试的类型是否是给定的类型
        {
            //"As" 会尝试将实例转换为给定的类型
            rs2::video_stream_profile video_stream_profile = stream_profile.as<rs2::video_stream_profile>();

            //使用“as”方法后，我们可以使用新的数据类型进行其他操作：
            std::cout << " (Video Stream: " << video_stream_profile.format() << " " << video_stream_profile.width() << "x" << video_stream_profile.height() << "@ " << video_stream_profile.fps() << "Hz)";
        }
        std::cout << std::endl;
        profile_num++;
    }

    uint32_t selected_profile_index = get_user_selection("Please select the desired streaming profile: ");
    if (selected_profile_index >= stream_profiles.size())
    {
        throw std::out_of_range("Requested profile index is out of range");
    }

    return stream_profiles[selected_profile_index];
}

void rs_camera::display_live_stream(rs2::device device, rs2::sensor sensor)
{
    // The rs2::sensor allows you to control its streams
    // We will first choose a single stream profile from the available profiles of the sensor
    rs2::stream_profile selected_profile = choose_a_streaming_profile(sensor);
    get_field_of_view(selected_profile);
    // Next, we will display the stream in a window
    start_streaming_a_profile(sensor, selected_profile);
}

void rs_camera::init_camera_with_opencv()
{
    rs2::config pipe_config;
    cv::FileStorage fs("../config/cameraParm.yaml", cv::FileStorage::READ);
    fs["RGB_width"] >> this->color_w;
    fs["RGB_height"] >> this->color_h;
    fs["depth_width"] >> this->depth_w;
    fs["depth_height"] >> this->depth_h;
    fs["infrared_width"] >> this->infrared_w;
    fs["infrared_height"] >> this->infrared_h;
    fs["RGB_fps"] >> this->color_fps;
    fs["depth_fps"] >> this->depth_fps;
    fs["infrared_fps"] >> this->infrared_fps;
    fs.release();
    pipe_config.enable_stream(RS2_STREAM_COLOR, color_w, color_h, RS2_FORMAT_BGR8, color_fps);
    // pipe_config.enable_stream(RS2_STREAM_DEPTH, depth_w, depth_h, RS2_FORMAT_Z16, depth_fps);
    // 以下是开IR图，就是得注掉上一行，目前不知道为什么IR和深度不能共存
    pipe_config.enable_stream(RS2_STREAM_INFRARED, 1, infrared_w, infrared_h, RS2_FORMAT_Y8, infrared_fps);
    pipe_config.enable_stream(RS2_STREAM_INFRARED, 2, infrared_w, infrared_h, RS2_FORMAT_Y8, infrared_fps);
    profile = pipe.start(pipe_config);
    rs2::device selected_device = profile.get_device();
    rs2::sensor color_sensor = selected_device.first<rs2::color_sensor>();
    rs2::sensor depth_sensor = selected_device.first<rs2::depth_sensor>();

    change_sensor_option(color_sensor, 1);
    change_sensor_option(depth_sensor, 2);
}

cv::Mat rs_camera::get_color_image()
{
    rs2::frame color_frame = frameset_.get_color_frame();
    cv::Mat color_image(cv::Size(color_w, color_h), CV_8UC3, (void *)color_frame.get_data(), cv::Mat::AUTO_STEP);
    return color_image;
}

cv::Mat rs_camera::get_depth_image()
{
    rs2::frame depth_frame = frameset_.get_depth_frame();
    cv::Mat depth_image(cv::Size(depth_w, depth_h), CV_16U, (void *)depth_frame.get_data(), cv::Mat::AUTO_STEP);
    return depth_image;
}

cv::Mat rs_camera::get_colorized_depth_image()
{
    rs2::frame depth_frame_4_show = frameset_.get_depth_frame().apply_filter(c);
    cv::Mat depth_image_4_show(cv::Size(depth_w, depth_h), CV_8UC3, (void *)depth_frame_4_show.get_data(), cv::Mat::AUTO_STEP);
    return depth_image_4_show;
}

cv::Mat rs_camera::get_left_ir_image()
{
    rs2::video_frame ir_frame_left = frameset_.get_infrared_frame(1);
    cv::Mat left_ir_image(cv::Size(infrared_w, infrared_h), CV_8UC1, (void *)ir_frame_left.get_data(), cv::Mat::AUTO_STEP);
    return left_ir_image;
}

cv::Mat rs_camera::get_right_ir_image()
{
    rs2::video_frame ir_frame_right = frameset_.get_infrared_frame(2);
    cv::Mat right_ir_image(cv::Size(infrared_w, infrared_h), CV_8UC1, (void *)ir_frame_right.get_data(), cv::Mat::AUTO_STEP);
    return right_ir_image;
}
