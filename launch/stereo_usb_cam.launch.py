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

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'io_method',
            default_value='shared_mem',
            description='ros/shared_mem'),
        DeclareLaunchArgument(
            'image_width',
            default_value='1280',
            description='camera out image width'),
        DeclareLaunchArgument(
            'image_height',
            default_value='720',
            description='camera out image height'),
        DeclareLaunchArgument(
            'enable_fb',
            default_value='False',
            description='enable publish local img'),
        DeclareLaunchArgument(
            'enable_dump',
            default_value='False',
            description='enable dump imgs from camera'),
        DeclareLaunchArgument(
            'video_device',
            default_value='0',
            description='video device index'),

        Node(
            package='hobot_stereo_usb_cam',
            executable='hobot_stereo_usb_cam',
            output='screen',
            parameters=[
                {"io_method": LaunchConfiguration('io_method')},
                {"video_device": LaunchConfiguration('video_device')},
                {"image_width": LaunchConfiguration('image_width')},
                {"image_height": LaunchConfiguration('image_height')},
                {"enable_fb": LaunchConfiguration('enable_fb')},
                {"enable_dump": LaunchConfiguration('enable_dump')}
            ],
            arguments=['--ros-args', '--log-level', 'warn']
        )
    ])
