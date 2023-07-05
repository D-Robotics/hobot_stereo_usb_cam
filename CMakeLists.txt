# Copyright (c) 2022，Horizon Robotics.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

cmake_minimum_required(VERSION 3.5)
project(hobot_stereo_usb_cam)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

add_compile_options(-w -ggdb)

add_definitions(-DVIDEO_MOD_AVAILABLE)

if(BUILD_HBMEM)
  message("BUILD_HBMEM is on")
  add_definitions(-DUSING_HBMEM)
  find_package(hbm_img_msgs REQUIRED)
else()
  message("BUILD_HBMEM is off")
endif()

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

## pkg-config libraries
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(yaml_cpp_vendor REQUIRED)

include_directories(
  include
  deps/include
)

link_directories(
  ${CMAKE_SYSROOT}/usr/lib/hbmedia/
  ${CMAKE_SYSROOT}/usr/lib/hbbpu/
  deps/lib
)

## Build the mipi camera library
add_executable(${PROJECT_NAME}
  src/main.cpp
  src/stereo_usb_cam_node.cpp
)

set(BASE_LIBRARIES pthread vio cam dl)

target_link_libraries(${PROJECT_NAME}
  ${BASE_LIBRARIES} yaml-cpp opencv_world
  zed_open_capture
  hidapi-libusb
  usb-1.0
)

ament_target_dependencies(
  ${PROJECT_NAME}
  rclcpp
  sensor_msgs
  yaml_cpp_vendor
  )

if(BUILD_HBMEM)
  target_link_libraries(${PROJECT_NAME}
    hbmem
  )
  ament_target_dependencies(
    ${PROJECT_NAME}
    rclcpp
    hbm_img_msgs
  )
endif()

install(DIRECTORY
  deps/lib/
  DESTINATION lib/
)

# Install executables
install(
  TARGETS ${PROJECT_NAME}
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  ${PROJECT_SOURCE_DIR}/config/
  DESTINATION lib/${PROJECT_NAME}/config/
)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
