English| [简体中文](./README_cn.md)

# Getting Started with Stereo_Cam Node
---

# Introduction

Obtain stereo image data from USB cameras and publish topic messages through `ROS2`. Supported camera type is `ZED 2i`.

# Supported Platforms

- RDK Ultra

# Development Environment

- Programming Language: C/C++
- Development Platform: RDK Ultra/X86
- System Version: Ubuntu 20.04
- Compilation Toolchain: Linux GCC 9.3.0/Linaro GCC 9.3.0

## Parameters

| Parameter | Description | Type | Supported Configurations | Mandatory | Default Value |
| --------- | ----------- | ---- | ------------------------ | --------- | ------------- |
| io_method | Communication method for publishing image data | string | ros/shared_mem | No | shared_mem |
| pub_topic_name | Topic name for publishing | string | Consistent with subscribed topic name | No | hbmem_stereo_img |
| image_height | Vertical resolution of image data | int | 1280 | No | 1280 |
| image_width | Horizontal resolution of image data | int | 720 | No | 720 |
| enable_fb | Publish image data in local NV12 format, image path is `frame_10_2560_720.nv12` under running path | bool | True/False | No | False |
| video_device | Device number | int | Configure based on the actual recognized device number | No | 0 |

## Compilation on RDK Ultra Ubuntu 20.04 System

1. Confirm the compilation environment

- tros.b is installed.

- ROS2 software package build system ament_cmake is installed. Installation command: `apt update; apt-get install python3-catkin-pkg; pip3 install empy`

- ROS2 compilation tool colcon is installed. Installation command: `pip3 install -U colcon-common-extensions`

- USB driver is installed. Installation command: `sudo apt install libusb-1.0-0-dev libhidapi-libusb0 libhidapi-dev`

2. Compilation

```shell
source /opt/tros/setup.bash
colcon build --packages-select hobot_stereo_usb_cam
```

## Cross-Compilation with Docker

1. Compilation Environment Verification

- Compile in Docker and tros.b has been compiled in Docker. For detailed instructions on Docker installation, cross-compilation, tros.b compilation, and deployment, please refer to [TogetheROS.Bot User Manual](https://developer.horizon.ai/api/v1/fileData/documents_tros/quick_start/cross_compile.html#).

2. Compilation

- Compilation command:

```shell
bash robot_dev_config/build.sh -p Rdkultra -s hobot_stereo_usb_cam
```

## Execution

```shell
source /opt/tros/setup.bash
ros2 launch hobot_stereo_usb_cam hobot_stereo_usb_cam.launch.py
```

After successful execution, the following log will be output:

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
