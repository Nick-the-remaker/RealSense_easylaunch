# RealSense_easylaunch
### 本代码为RealSense快速上手程序，供刚接触RealSense或想在项目里嵌入相机进行开发的来使用，代码实质是根据官方例程的api_how_to.h进行的二次封装。
### 环境配置：不依赖ROS，有RealSense官方说明的环境依赖和OpenCV即可。
### 开发方式，提供OpenCV和OpenGL两种开发方式，通过添加宏定义 USE_OPENCV 来切换。OpenCV提供彩色图，深度图，红外图的接口，目前还不够全面，后续会持续更新。
### 编译：
```
cd RealSense_easylaunch
mkdir build;
cd build;
cmake ..;
make -j;
./RealSense_camera;
```