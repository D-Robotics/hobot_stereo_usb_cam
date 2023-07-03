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

#include <string>
#include <memory>
#include <unistd.h>

#include "rclcpp/rclcpp.hpp"
#include "stereo_usb_cam_node.h"

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);

  rclcpp::spin(std::make_shared<stereo_usb_cam::StereoUsbCamNode>("hobot_stereo_mipi_cam"));
  
  // auto sp_node = std::make_shared<stereo_usb_cam::StereoUsbCamNode>("hobot_stereo_mipi_cam");
  // rclcpp::executors::MultiThreadedExecutor exec;
  // // rclcpp::executors::MultiThreadedExecutor exec(
  // //     rclcpp::ExecutorOptions(), 8
  // //   );
  // printf("\n\n get_number_of_threads: %d \n\n", exec.get_number_of_threads());
  // exec.add_node(sp_node);
  // exec.spin();

  rclcpp::shutdown();
  return 0;
}
