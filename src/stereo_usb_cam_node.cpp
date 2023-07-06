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

#include <sstream>
#include <memory>
#include <string>
#include <vector>
#include <fstream>
#include <unistd.h>

#include <stdio.h>
#include <termios.h>

#include "opencv2/core/mat.hpp"
#include "opencv2/imgcodecs.hpp"
#include "opencv2/imgproc.hpp"

#include "stereo_usb_cam_node.h"

#include "ocv_display.hpp"

namespace stereo_usb_cam {

StereoUsbCamNode::StereoUsbCamNode(const std::string& node_name)
    : is_init_(false), Node(node_name, rclcpp::NodeOptions()) {
  if (GetParams() < 0) {
    RCLCPP_ERROR_STREAM(rclcpp::get_logger("stereo_usb_cam_node"), "Get params failed!");
    rclcpp::shutdown();
    return;
  }

  if (CheckParams() < 0) {
    RCLCPP_ERROR_STREAM(rclcpp::get_logger("stereo_usb_cam_node"), "Check params failed!");
    rclcpp::shutdown();
    return;
  }

  if (Init() < 0) {
    RCLCPP_ERROR_STREAM(rclcpp::get_logger("stereo_usb_cam_node"), "Init failed!");
    rclcpp::shutdown();
    return;
  }

  if (enable_fb_) {
    fb_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(static_cast<int64_t>(30)),
        std::bind(&StereoUsbCamNode::Feedback, this));
  }
}

StereoUsbCamNode::~StereoUsbCamNode() {
  dump_img_task_cv_.notify_all();
  if (sp_dumptask_) {
    sp_dumptask_->join();
  }
  RCLCPP_WARN(rclcpp::get_logger("stereo_usb_cam_node"), "shutting down");
}

int StereoUsbCamNode::GetParams() {
  // declare params
  this->declare_parameter("camera_name", "default_cam");
  this->declare_parameter("framerate", framerate_);
  this->declare_parameter("frame_id", "default_cam");
  this->declare_parameter("image_height", image_height_);
  this->declare_parameter("image_width", image_width_);
  this->declare_parameter("io_method", io_method_);
  this->declare_parameter("pub_topic_name", pub_hbmem_topic_name_);
  this->declare_parameter("out_format", out_format_);
  this->declare_parameter("video_device", video_device_);
  this->declare_parameter("enable_fb", enable_fb_);
  this->declare_parameter("enable_dump", enable_dump_);

  this->get_parameter<std::string>("camera_name", camera_name_);
  this->get_parameter<int>("framerate", framerate_);
  this->get_parameter<int>("image_height", image_height_);
  this->get_parameter<int>("image_width", image_width_);
  this->get_parameter<std::string>("io_method", io_method_);
  this->get_parameter<std::string>("pub_topic_name", pub_hbmem_topic_name_);
  this->get_parameter<std::string>("out_format", out_format_);
  this->get_parameter<int>("video_device", video_device_);
  this->get_parameter<bool>("enable_fb", enable_fb_);
  this->get_parameter<bool>("enable_dump", enable_dump_);

  RCLCPP_WARN_STREAM(
      rclcpp::get_logger("stereo_usb_cam_node"),
      "Get params complete."
      << "\n camera_name: " << camera_name_
      << "\n video_device index: " << video_device_
      << "\n image_width: " << image_width_
      << "\n image_height: " << image_height_
      << "\n io_method_name: " << io_method_
      << "\n pub_topic_name: " << pub_hbmem_topic_name_
      << "\n out_format: " << out_format_
      << "\n enable_fb: " << enable_fb_
      << "\n enable_dump: " << enable_dump_
    );

  return 0;
}

int StereoUsbCamNode::CheckParams() {
  if (io_method_ != "ros" && io_method_ != "shared_mem") {
    RCLCPP_ERROR_STREAM(rclcpp::get_logger("stereo_usb_cam_node"),
      "Invalid input io_method_name: " << io_method_ << ", which should be \"ros\"/\"shared_mem\".");
    return -1;
  }

  return 0;
}

int StereoUsbCamNode::Init() {
  if (is_init_) return 0;

  if (io_method_.compare("ros") == 0) {
    image_pub_ =
      this->create_publisher<sensor_msgs::msg::Image>("image_raw", qos_depth_);
  } else {
#ifdef USING_HBMEM
    // 创建hbmempub
    publisher_hbmem_ =
        this->create_publisher_hbmem<hbm_img_msgs::msg::HbmMsg1080P>(
            pub_hbmem_topic_name_, qos_depth_);
#else
    RCLCPP_ERROR_STREAM(rclcpp::get_logger("stereo_usb_cam_node"),
                      "Hbmem is not enabled.");
#endif
  }

  if (enable_fb_) {
    RCLCPP_WARN_STREAM(rclcpp::get_logger("stereo_usb_cam_node"),
                      "fb is enabled.");
    is_init_ = true;
    return 0;
  }

  // 创建图像获取任务
  auto get_img = [this](){
    
    sl_oc::video::VideoParams params;
    params.res = sl_oc::video::RESOLUTION::HD720;
    params.fps = sl_oc::video::FPS::FPS_30;
    params.verbose = sl_oc::VERBOSITY::INFO;

    // ----> Create Video Capture
    sl_oc::video::VideoCapture cap_0(params);
    if( !cap_0.initializeVideo(video_device_) )
    {
      RCLCPP_ERROR_STREAM(rclcpp::get_logger("stereo_usb_cam_node"), 
          "Cannot open camera video capture!" << "\n" << "See verbosity level for more details.");
      rclcpp::shutdown();
      return;
    }

    RCLCPP_WARN_STREAM(rclcpp::get_logger("stereo_usb_cam_node"), 
        "Open video device " << video_device_ << " success.\n"
        << "camera sn: " << cap_0.getSerialNumber() << "[" << cap_0.getDeviceName() << "]");

    while (rclcpp::ok()) {
      // Get last available frame
      const sl_oc::video::Frame frame = cap_0.getLastFrame();

      RCLCPP_DEBUG_STREAM(rclcpp::get_logger("stereo_usb_cam_node"), 
        "Get frame w: " << frame.width
        << ", h: " << frame.height
        << ", frame_id: " << frame.frame_id
        << ", timestamp: " << frame.timestamp
      );
      
      if (frame.data!=nullptr) {
        Publish(frame);
      } else {
        RCLCPP_ERROR_STREAM(rclcpp::get_logger("stereo_usb_cam_node"), 
          "Frame data is invalid");
      }
    }
  };

  sp_getimg_task_ = std::make_shared<std::thread>([this, get_img](){
    get_img();
  });

  // timer_ = this->create_wall_timer(
  //     std::chrono::milliseconds(static_cast<int64_t>(10)), get_img);

  is_init_ = true;
  return 0;
}

int StereoUsbCamNode::Publish(const sl_oc::video::Frame& frame) {
  if (!publisher_hbmem_) {
    RCLCPP_ERROR_STREAM(rclcpp::get_logger("stereo_usb_cam_node"), 
         "publisher_hbmem_ is invalid!");
  }

  auto loanedMsg = publisher_hbmem_->borrow_loaned_message();
  if (!loanedMsg.is_valid()) {
    RCLCPP_INFO(rclcpp::get_logger("stereo_usb_cam_node"),
                  "borrow_loaned_message fail!");
    return -1;
  }

  auto& msg = loanedMsg.get();

  // 转成nv12后发布
  int dest_size = frame.width * frame.height * 3 / 2;
  unsigned char *dest = msg.data.data();
  // unsigned char *dest = 
  //         reinterpret_cast<unsigned char *>(calloc(1, dest_size));
  assert(dest != NULL);
  if (yuyv_to_nv12(frame.data, dest, frame.width, frame.height, frame.width * frame.height * frame.channels) < 0) {
    std::cout << "convert data from yuy2 to nv12 failed...\n";
    return -1;
  }

  if (enable_dump_) {
    if (frame.frame_id % 10 == 0) {
      // // dump yuv422
      // {
      //   std::ofstream ofs("frame_" + std::to_string(frame.frame_id)
      //     + "_" +  std::to_string(frame.width)
      //     + "_" + std::to_string(frame.height)
      //       + ".yuv422");
      //   int dat_len = frame.height * frame.width * frame.channels;
      //   ofs.write((const char*)frame.data, dat_len);
      // }

      // dump nv12
      {
        std::ofstream ofs("frame_" + std::to_string(frame.frame_id)
          + "_" +  std::to_string(frame.width)
          + "_" + std::to_string(frame.height)
            + ".nv12");
        ofs.write((const char*)dest, dest_size);
      }

      // // dump jpg
      // {
      //   cv::Mat nv12(frame.height * 3 / 2, frame.width, CV_8UC1, (char*)dest);
      //   cv::Mat bgr_mat;
      //   cv::cvtColor(nv12, bgr_mat, CV_YUV2BGR_NV12);  //  nv12 to bgr
      //   cv::imwrite("frame_" + std::to_string(frame.frame_id)
      //       + "_" +  std::to_string(frame.width)
      //       + "_" + std::to_string(frame.height)
      //       + ".jpg", bgr_mat);
      // }


      // dump left img
      {
        int w = frame.width / 2;
        int h = frame.height;
        unsigned char *buf = reinterpret_cast<unsigned char *>(calloc(1, w*h*1.5));
        unsigned char *tmp_src = dest;
        unsigned char *tmp_buf = buf;
        for (int idx_h = 0; idx_h < frame.height; idx_h++) {
          memcpy(tmp_buf, tmp_src, w);
          tmp_buf+= w;
          tmp_src+= frame.width;
        }
        for (int idx_h = 0; idx_h < frame.height / 2; idx_h++) {
          memcpy(tmp_buf, tmp_src, w);
          tmp_buf+= w;
          tmp_src+= frame.width;
        }

        std::ofstream ofs("frame_" + std::to_string(frame.frame_id)
            + "_" +  std::to_string(w)
            + "_" + std::to_string(h)
            + "_left.nv12");
        ofs.write((const char*)buf, w*h*1.5);

        cv::Mat nv12(h * 3 / 2, w, CV_8UC1, (char*)buf);
        cv::Mat bgr_mat;
        cv::cvtColor(nv12, bgr_mat, CV_YUV2BGR_NV12);  //  nv12 to bgr
        cv::imwrite("frame_" + std::to_string(frame.frame_id)
            + "_" +  std::to_string(w)
            + "_" + std::to_string(h)
            + "_left.jpg", bgr_mat);

        free(buf);
      }

      // dump right img
      {
        int w = frame.width / 2;
        int h = frame.height;
        unsigned char *buf = reinterpret_cast<unsigned char *>(calloc(1, w*h*1.5));
        unsigned char *tmp_src = dest;
        unsigned char *tmp_buf = buf;
        for (int idx_h = 0; idx_h < frame.height; idx_h++) {
          tmp_src += w;
          memcpy(tmp_buf, tmp_src, w);
          tmp_buf += w;
          tmp_src += w;
        }
        for (int idx_h = 0; idx_h < frame.height / 2; idx_h++) {
          tmp_src += w;
          memcpy(tmp_buf, tmp_src, w);
          tmp_buf += w;
          tmp_src += w;
        }

        std::ofstream ofs("frame_" + std::to_string(frame.frame_id)
            + "_" +  std::to_string(w)
            + "_" + std::to_string(h)
            + "_right.nv12");
        ofs.write((const char*)buf, w*h*1.5);

        cv::Mat nv12(h * 3 / 2, w, CV_8UC1, (char*)buf);
        cv::Mat bgr_mat;
        cv::cvtColor(nv12, bgr_mat, CV_YUV2BGR_NV12);  //  nv12 to bgr
        cv::imwrite("frame_" + std::to_string(frame.frame_id)
            + "_" +  std::to_string(w)
            + "_" + std::to_string(h)
            + "_right.jpg", bgr_mat);

        free(buf);
      }
    }
  }
  
  msg.time_stamp.sec = frame.timestamp / 1e9;
  msg.time_stamp.nanosec = frame.timestamp - msg.time_stamp.sec * 1e9;
  memcpy(msg.encoding.data(), "nv12", strlen("nv12"));
  msg.height = frame.height;
  msg.width = frame.width;
  msg.step = frame.width;
  msg.data_size = dest_size;
  msg.index = frame.frame_id;
  // memcpy(msg.data.data(), dest, data_size);

  publisher_hbmem_->publish(std::move(loanedMsg));

  RCLCPP_INFO(rclcpp::get_logger("stereo_usb_cam_node"),
                "pub from cam with topic %s", pub_hbmem_topic_name_.data());
                
  return 0;
}

int StereoUsbCamNode::Feedback() {
  if (!publisher_hbmem_) {
    RCLCPP_ERROR_STREAM(rclcpp::get_logger("stereo_usb_cam_node"), 
         "publisher_hbmem_ is invalid!");
  }

  std::string nv12_fname = "frame_10_2560_720.nv12";
  int w = 2560;
  int h = 720;
  
  std::ifstream ifs(nv12_fname);
  if (!ifs) {
    RCLCPP_ERROR_STREAM(rclcpp::get_logger("stereo_usb_cam_node"), 
         "open nv12_fname fail! " << nv12_fname);
  }

  auto loanedMsg = publisher_hbmem_->borrow_loaned_message();
  if (!loanedMsg.is_valid()) {
    RCLCPP_INFO(rclcpp::get_logger("stereo_usb_cam_node"),
                  "fb borrow_loaned_message fail!");
    return -1;
  }

  auto& msg = loanedMsg.get();

  // 转成nv12后发布
  int dest_size = w * h * 3 / 2;
  char *dest = (char*)msg.data.data();
  assert(dest != NULL);
  ifs.read(dest, dest_size);

  // msg.time_stamp.sec = frame.timestamp / 1e9;
  // msg.time_stamp.nanosec = frame.timestamp - msg.time_stamp.sec * 1e9;
  memcpy(msg.encoding.data(), "nv12", strlen("nv12"));
  msg.height = h;
  msg.width = w;
  msg.step = w;
  msg.data_size = dest_size;
  // msg.index = frame.frame_id;

  publisher_hbmem_->publish(std::move(loanedMsg));

  RCLCPP_INFO_STREAM(rclcpp::get_logger("stereo_usb_cam_node"),
                "fb pub from file: " << nv12_fname
                << ", h: " << h
                << ", w: " << w
                << ", with topic " << pub_hbmem_topic_name_);
                
  return 0;
}

int StereoUsbCamNode::yuyv_to_nv12(uint8_t * image_in, uint8_t* image_out, int width, int height, unsigned long int filesize) {
  if (!image_in || !image_out || width <= 0 || height <= 0) {
    RCLCPP_ERROR_STREAM(rclcpp::get_logger("stereo_usb_cam_node"), 
         "some error happen... in_frame:" << image_in
         << "  out_frame:" << image_out
         << "  width: " << width
         << "  height:" << height);
    return -1;
  }

	unsigned int pixNUM = (width<<1);	  
	unsigned int j =0,k =0;
	
	uint8_t *uv=image_out + width*height;
	uint8_t *u8start=image_in-(width<<1);
	uint8_t *y = image_out;
	uint16_t *u16start;
	
	pixNUM = pixNUM-16;
	for(j=0; j< height; j+=2)  //
	{
		//偶数行 提取 Y，UV
		u8start += (width<<1);
		for(k = 0; k<pixNUM; k=k+16)    //YUYV
		{
			*y++ = *u8start++;
			*uv++= *u8start++;
			*y++ = *u8start++;
			*uv++ = *u8start++;//4

			*y++ = *u8start++;
			*uv++= *u8start++;
			*y++ = *u8start++;
			*uv++ = *u8start++;//8
			
			*y++ = *u8start++;
			*uv++= *u8start++;
			*y++ = *u8start++;
			*uv++ = *u8start++;//12
			
			*y++ = *u8start++;
			*uv++= *u8start++;
			*y++ =  *u8start++;
			*uv++ = *u8start++;//16
		}
		for(; k< pixNUM+16; k=k+4)    //YUYV
		{
			*y++ = *u8start++;
			*uv++= *u8start++;
			*y++ = *u8start++;
			*uv++= *u8start++;
			
		}
		
		//奇数数行提取 Y
		u16start = (uint16_t *)u8start;
		for(k = 0; k<width-8; k=k+8)    //YUYV
		{
			*y++= *u16start++;
			*y++= *u16start++;
			*y++= *u16start++;
			*y++= *u16start++;//4			
			*y++= *u16start++;
			*y++= *u16start++;			
			*y++= *u16start++;
			*y++= *u16start++;//8
		}
		for(; k< width; k++)    //YUYV
		{
			*y++= *u16start++;
			
		}
	} 

  return 0;
 }

int StereoUsbCamNode::Getch() {
  int ch;
  struct termios oldt;
  struct termios newt;

  // Store old settings, and copy to new settings
  tcgetattr(STDIN_FILENO, &oldt);
  newt = oldt;

  // Make required changes and apply the settings
  newt.c_lflag &= ~(ICANON | ECHO);
  newt.c_iflag |= IGNBRK;
  newt.c_iflag &= ~(INLCR | ICRNL | IXON | IXOFF);
  newt.c_lflag &= ~(ICANON | ECHO | ECHOK | ECHOE | ECHONL | ISIG | IEXTEN);
  newt.c_cc[VMIN] = 1;
  newt.c_cc[VTIME] = 0;
  tcsetattr(fileno(stdin), TCSANOW, &newt);

  // Get the current character
  ch = getchar();

  // Reapply old settings
  tcsetattr(STDIN_FILENO, TCSANOW, &oldt);

  return ch;
}

}  // namespace stereo_usb_cam
