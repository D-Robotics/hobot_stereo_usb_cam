# Getting Started with hobot_stereo_cam Node
---


## 功能

从ZED 2i双目相机采集图像，并且通过零拷贝通信方式，以`hbmem_stereo_img`话题发布2560x720分辨率的NV12格式的图片数据。

## 平台

地平线RDK X3, RDK X3 Module和RDK J5。


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