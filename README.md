# Getting Started with Stereo_Cam Node
---

# 功能介绍

从USB摄像头获取双目图像数据并通过话题发布。

# 支持平台

- RDK J5

# 开发环境

- 编程语言: C/C++
- 开发平台: RDK J5/X86
- 系统版本：Ubuntu 20.04
- 编译工具链:Linux GCC 9.3.0/Linaro GCC 9.3.0

## 参数

| 参数名      | 解释             | 类型   | 支持的配置                 | 是否必须 | 默认值             |
| ------------| -----------------| -------| --------------------------| -------- | -------------------|
| io_method| 发布图像数据的通信方式 | string    | ros/shared_mem         | 否       | shared_mem                |
| pub_topic_name| 发布话题名 | string    | 和订阅的话题名一致         | 否       | hbmem_stereo_img                |
| image_height| 图像数据的高方向分辨率 | int    | 1280         | 否       | 1280                |
| image_width | 图像数据的宽方向分辨率 | int    | 720         | 否        | 720               |
| enable_fb   | 发布本地NV12格式的图像数据，图片路径为运行路径下的`frame_10_2560_720.nv12`文件  | bool | True/False | 否 | False |
| video_device | 设备号 | int    | 根据实际识别出来的设备号配置         | 否        | 0               |

## RDK J5 Ubuntu 20.04系统上编译

1、编译环境确认

- 已安装tros.b。

- 已安装ROS2软件包构建系统ament_cmake。安装命令：`apt update; apt-get install python3-catkin-pkg; pip3 install empy`

- 已安装ROS2编译工具colcon。安装命令：`pip3 install -U colcon-common-extensions`

- 已安装USB驱动。安装命令：`sudo apt install libusb-1.0-0-dev libhidapi-libusb0 libhidapi-dev`

2、编译

```shell
source /opt/tros/setup.bash
colcon build --packages-select hobot_stereo_usb_cam --cmake-args -DBUILD_HBMEM=ON
```

## 运行

```shell
source /opt/tros/setup.bash
ros2 launch hobot_stereo_usb_cam stereo_usb_cam.launch.py
```

运行成功后输出如下log:

```shell
[INFO] [launch]: All log files can be found below /root/.ros/log/2023-07-04-22-45-35-530438-hobot-2618037
[INFO] [launch]: Default logging verbosity is set to INFO
[INFO] [hobot_stereo_usb_cam-1]: process started with pid [2618039]
[hobot_stereo_usb_cam-1] [WARN] [1688510736.176433788] [stereo_usb_cam_node]: Get params complete.
[hobot_stereo_usb_cam-1]  camera_name: default_cam
[hobot_stereo_usb_cam-1]  video_device index: 0
[hobot_stereo_usb_cam-1]  image_width: 1280
[hobot_stereo_usb_cam-1]  image_height: 720
[hobot_stereo_usb_cam-1]  io_method_name: shared_mem
[hobot_stereo_usb_cam-1]  out_format: nv12
[hobot_stereo_usb_cam-1]  enable_fb: 0
[hobot_stereo_usb_cam-1]  enable_dump: 0
[hobot_stereo_usb_cam-1] [sl_oc::video::VideoCapture] INFO: ZED Open Capture - Camera module - Version: 0.6.0
[hobot_stereo_usb_cam-1] [sl_oc::video::VideoCapture] INFO: Camera resolution: 2560x720@30Hz
[hobot_stereo_usb_cam-1] [sl_oc::video::VideoCapture] INFO: Trying to open the device '/dev/video0'
[hobot_stereo_usb_cam-1] [sl_oc::video::VideoCapture] INFO: Opened camera with SN: 38085162
[hobot_stereo_usb_cam-1] [sl_oc::video::VideoCapture] INFO: Device '/dev/video0' opened
[hobot_stereo_usb_cam-1] [WARN] [1688510736.528142580] [stereo_usb_cam_node]: Open video device 0 success.
[hobot_stereo_usb_cam-1] camera sn: 38085162[/dev/video0]
```
