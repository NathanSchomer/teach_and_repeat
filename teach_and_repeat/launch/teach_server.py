# teach_server.py
#
# Nathan Schomer <nathans@glidance.io>

from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package="teach_and_repeat",
                executable="teach_server",
                name="teach_server",
                parameters=[{
                    "img_color_topic": "/camera_down/color/image_raw_rfps",
                    "img_depth_topic": "/camera_down/depth/image_raw_rfps",
                    "odom_topic": "/odom"        
                             }],
            ),
        ]
    )
