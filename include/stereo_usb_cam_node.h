// Copyright (c) 2022，Horizon Robotics.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef MIPI_STEREO_CAM_NODE_H_
#define MIPI_STEREO_CAM_NODE_H_
#include "stereo_usb_cam_node.h"

#include <rclcpp/rclcpp.hpp>

#include <memory>
#include <string>

#include "sensor_msgs/image_encodings.hpp"
#include "sensor_msgs/msg/compressed_image.hpp"

#include "sensor_msgs/msg/image.hpp"
#include "std_srvs/srv/set_bool.hpp"
#include <std_msgs/msg/string.hpp>

#include "videocapture.hpp"

#ifdef USING_HBMEM
// #include "hb_mem_mgr.h"
#include "hbm_img_msgs/msg/hbm_msg1080_p.hpp"
#endif

namespace stereo_usb_cam
{
class StereoUsbCamNode : public rclcpp::Node
{
public:
  StereoUsbCamNode(const std::string& node_name);
  ~StereoUsbCamNode();

private:
  int GetParams();
  int CheckParams();

  int Init();
  
  // For non-blocking keyboard inputs
  int Getch();

  int Feedback();

  // yuv422转nv12
  int yuyv_to_nv12(uint8_t * image_in, uint8_t* image_out, int width, int height, unsigned long int filesize);

  int Publish(const sl_oc::video::Frame& frame);

  rclcpp::TimerBase::SharedPtr fb_timer_ = nullptr;

  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub_ = nullptr;

  std::shared_ptr<std::thread> sp_getimg_task_ = nullptr;

  std::shared_ptr<std::thread> sp_dumptask_ = nullptr;
  std::shared_ptr<std::thread> sp_teleop_task_ = nullptr;

#ifdef USING_HBMEM
  int32_t mSendIdx = 0;
  rclcpp::TimerBase::SharedPtr timer_hbmem_;
  rclcpp::PublisherHbmem<hbm_img_msgs::msg::HbmMsg1080P>::SharedPtr publisher_hbmem_;
  std::string pub_hbmem_topic_name_ = "hbmem_stereo_img";
#endif
/*
  sensor_msgs::msg::CompressedImage::SharedPtr ros_img_compressed_;
  rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr video_compressed_publisher_;
*/
  // parameters
  int video_device_ = 0;
  std::string frame_id_;
  std::string io_method_ = "shared_mem";  // "shared_mem"/"ros"
  std::string imgMsgPtr;
  int image_width_ = 1280;
  int image_height_ = 720;
  int framerate_ = 30;
  std::string out_format_ = "nv12";
  bool enable_fb_ = false;
  bool enable_dump_ = false;

  std::atomic_bool is_init_;

  std::string camera_name_;
  std::string camera_info_url_;
  std::string camera_calibration_file_path_;

  rclcpp::TimerBase::SharedPtr timer_ = nullptr;
  const int qos_depth_ = 10;
  
  size_t cache_len_limit_ = 10;
  std::vector<int> video_index_ {0, 1};
  std::map<int, uint64_t> counts_;
  uint64_t get_img_counts_ = 0;

  std::mutex img_mtx_;

  std::queue<std::function<void()>> dump_img_task_cache_;

  std::mutex dump_img_task_mtx_;
  std::condition_variable dump_img_task_cv_;

  std::string data_collecting_path_ = "./data_collecting/";
  
  int data_sampling_rate_ = 30;
  uint64_t dump_count_ = 0;
};
}  // namespace stereo_usb_cam
#endif  // MIPI_STEREO_CAM_NODE_H_
