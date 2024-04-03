# Changelog for package hobot_stereo_usb_cam

tros_2.1.0 (2024-04-01)
------------------
1. 适配ros2 humble零拷贝。
2. 新增中英双语README。
3. 零拷贝通信使用的qos的Reliability由RMW_QOS_POLICY_RELIABILITY_RELIABLE（rclcpp::QoS()）变更为RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT（rclcpp::SensorDataQoS()）。

tros_2.0.1 (2023-07-14)
------------------
1. 规范Rdkultra产品名。

tros_2.0.0 (2023-07-07)
------------------
1. 支持ZED 2i Stereo Camera模组。
