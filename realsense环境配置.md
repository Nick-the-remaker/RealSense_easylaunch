该功能包涉及深度相机的调用，需要配置深度相机realsense-D455相机的环境
进入该网址
https://github.com/IntelRealSense/librealsense/blob/master/doc/distribution_linux.md
（该网址提供了环境配置的步骤，以下步骤和官网一样）

1.//注册服务器的公用密钥：
sudo apt-key adv --keyserver keys.gnupg.net --recv-key F6E65AC044F831AC80A06380C8B3A55A6F3EFCDE || sudo apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv-key F6E65AC044F831AC80A06380C8B3A55A6F3EFCDE

2.//将服务器添加到存储库列表中
Ubuntu 16 LTS：
sudo add-apt-repository "deb http://realsense-hw-public.s3.amazonaws.com/Debian/apt-repo xenial main" -u
//或者按下面这个来
Add the server to the list of repositories:
Ubuntu 16 LTS:
sudo add-apt-repository "deb https://librealsense.intel.com/Debian/apt-repo xenial main" -u
Ubuntu 18 LTS:
sudo add-apt-repository "deb https://librealsense.intel.com/Debian/apt-repo bionic main" -u
Ubuntu 20 LTS:
sudo add-apt-repository "deb https://librealsense.intel.com/Debian/apt-repo focal main" -u

3.//安装库（如果要升级软件包，请参见下面的部分）
sudo apt-get install librealsense2-dkms
sudo apt-get install librealsense2-utils

4.//可选安装开发和调试包
sudo apt-get install librealsense2-dev
sudo apt-get install librealsense2-dbg

5.重新连接英特尔实感深度摄像头，然后运行：realsense-viewer验证安装。
realsense-viewer 运行成功，则环境配置完成。

6.安装glfw插件 sudo apt install libglfw3或者sudo apt install libglfw3-dev

7.安装GLUT  sudo apt-get install freeglut3-dev 或者 sudo apt-get install libglut-dev

英特尔深度相机示例代码https://dev.intelrealsense.com/docs/code-samples?_ga=2.40280033.219234763.1620224200-92814120.1618399150