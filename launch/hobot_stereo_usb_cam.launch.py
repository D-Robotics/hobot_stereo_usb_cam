# Copyright (c) 2024，D-Robotics.
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
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python import get_package_share_directory
import os


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'io_method',
            default_value='shared_mem',
            description='ros/shared_mem'),
        DeclareLaunchArgument(
            'pub_topic_name',
            default_value='hbmem_stereo_img'),
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

        # 启动零拷贝环境配置node
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(
                    get_package_share_directory('hobot_shm'),
                    'launch/hobot_shm.launch.py'))
        ),
        Node(
            package='hobot_stereo_usb_cam',
            executable='hobot_stereo_usb_cam',
            output='screen',
            parameters=[
                {"io_method": LaunchConfiguration('io_method')},
                {"pub_topic_name": LaunchConfiguration('pub_topic_name')},
                {"video_device": LaunchConfiguration('video_device')},
                {"image_width": LaunchConfiguration('image_width')},
                {"image_height": LaunchConfiguration('image_height')},
                {"enable_fb": LaunchConfiguration('enable_fb')},
                {"enable_dump": LaunchConfiguration('enable_dump')}
            ],
            arguments=['--ros-args', '--log-level', 'warn']
        )
    ])
