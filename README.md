# Getting Started with Stereo_Cam Node
---


## 参数

| 参数名      | 解释             | 类型   | 支持的配置                 | 是否必须 | 默认值             |
| ------------| -----------------| -------| --------------------------| -------- | -------------------|
| image_height| 采集图像数据的高方向分辨率 | int    | 根据sensor支持选择         | 否       | 1280                |
| image_width | 采集图像数据的宽方向分辨率 | int    | 根据sensor支持选择         | 否        | 720               |
| data_sampling_rate   | 采集图像数据的帧率。<=0表示不使能采集功能，>0表示每data_sampling_rate帧采集一帧图像数据  | int | 无限制 | 否 | 30 |

## 编译

```shell
sudo apt install libusb-1.0-0-dev libhidapi-libusb0 libhidapi-dev

source /opt/tros/setup.bash
colcon build --packages-select hobot_stereo_usb_cam --cmake-args -DBUILD_HBMEM=ON
```

## 运行

```shell
source /opt/tros/setup.bash
source install/local_setup.bash
ros2 launch hobot_stereo_usb_cam stereo_usb_cam.launch.py
```